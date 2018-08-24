extern crate cgmath;
#[macro_use]
extern crate clap;
extern crate collision;
extern crate futures;
extern crate grpcio;
extern crate point_viewer;
extern crate point_viewer_grpc;
extern crate protobuf;

use cgmath::{Frustum, Matrix4, PerspectiveFov, Point3, Transform}
use collision::Aabb3;
use futures::{Stream, Future, Sink};
use futures::sync::oneshot;
use grpcio::{Environment, RpcContext, ServerBuilder, ServerStreamingSink, UnarySink, WriteFlags};
use point_viewer::InternalIterator;
use point_viewer::octree::{read_meta_proto, NodeId, Octree, OnDiskOctree};
use point_viewer_grpc::proto;
use point_viewer_grpc::proto_grpc;
use protobuf::Message;
use std::{io, thread};
use std::io::Read;
use std::path::PathBuf;
use std::sync::Arc;
use futures::sync::mpsc;

#[derive(Clone)]
struct OctreeService {
    octree: Arc<OnDiskOctree>,
    meta: point_viewer::proto::Meta,
}

impl proto_grpc::Octree for OctreeService {
    fn get_meta(
        &self,
        ctx: RpcContext,
        req: proto::GetMetaRequest,
        sink: UnarySink<proto::GetMetaReply>,
    ) {
        let mut resp = proto::GetMetaReply::new();
        resp.set_meta(self.meta.clone());
        let f = sink.success(resp)
            .map_err(move |e| println!("failed to reply {:?}: {:?}", req, e));
        ctx.spawn(f)
    }

    fn get_node_data(
        &self,
        ctx: RpcContext,
        req: proto::GetNodeDataRequest,
        sink: UnarySink<proto::GetNodeDataReply>,
    ) {
        let data = self.octree
            .get_node_data(&NodeId::from_str(&req.id))
            .unwrap();
        let mut resp = proto::GetNodeDataReply::new();
        resp.mut_node()
            .set_position_encoding(data.meta.position_encoding.to_proto());
        resp.mut_node().set_num_points(data.meta.num_points);
        resp.set_position(data.position);
        resp.set_color(data.color);
        let f = sink.success(resp)
            .map_err(move |e| println!("failed to reply {:?}: {:?}", req, e));
        ctx.spawn(f)
    }

    fn get_points_in_frustum(
        &self,
        ctx: RpcContext,
        req: proto::GetPointsInFrustumRequest,
        resp: ServerStreamingSink<proto::GetPointsInFrustumReply>,
    ) {
        let projection_matrix = Matrix4::from(PerspectiveFov {
            fov_y: req.fov_y,
            aspect: req.aspect,
            z_near:req.z_near,
            z_far: req.z_far,
        });
        let view_position = Point3::new(req.view_position.x, req.view_position.y, req.view_position.z);
        let view_direction = Vector3::new(req.view_direction.x, req.view_direction.y, req.view_direction.z);
        let view_up = Vector3::new(req.view_up.x, req.view_up.y, req.view_up.z);
        let view_center = view_position + view_up;

        let view_transform = Transform.look_at(view_position, view_center, view_up);
        let view_matrix: Matrix4<f32> = view_transform.inverse_transform().unwrap().into();
        let frustum_matrix = projection_matrix * view_matrix;

        let octree = self.octree.clone();
        let now = ::std::time::Instant::now();
        let visible_nodes = octree.get_visible_nodes(&frustum_matrix);
        println!(
            "Currently visible nodes: {}, time to calculate: {:?}",
            visible_nodes.len(),
            now.elapsed()
        );

        let mut reply = proto::GetPointsInFrustumReply::new();

        let frustum = Frustum::from_matrix4(projection_matrix);
        visible_nodes.for_each(|node| {
            let node_iterator = match octree::NodeIterator::from_disk(self.meta, &node) {
                Ok(node_iterator) => node_iterator,
                Err(Error(ErrorKind::NodeNotFound, _)) => continue,
                Err(err) => return Err(err),
            };

            let mut points = Vec::with_capacity(node_iterator.size_hint().unwrap());
            node_iterator.for_each(|p| points.push((*p).clone()));
            points.for_each(|p| {
                let child_relation = frustum.contains(&p);
                if child_relation == Relation::In {
                        let mut v = point_viewer::proto::Vector3f::new();
                        v.set_x(p.position.x);
                        v.set_y(p.position.y);
                        v.set_z(p.position.z);
                        reply.mut_points().push(v);
                }
            });
        }
        reply
    }

    fn get_points_in_box(
        &self,
        ctx: RpcContext,
        req: proto::GetPointsInBoxRequest,
        resp: ServerStreamingSink<proto::GetPointsInBoxReply>,
    ) {
        use std::thread;

        // This creates a async-aware (tx, rx) pair that can wake up the event loop when new data
        // is piped through it.
        let (tx, rx) = mpsc::channel(4);
        let octree = self.octree.clone();
        thread::spawn(move || {
            // This is the secret sauce connecting an OS thread to a event-based receiver. Calling
            // wait() on this turns the event aware, i.e. async 'tx' into a blocking 'tx' that will
            // make this thread block when the event loop is not quick enough with piping out data.
            let mut tx = tx.wait();

            let bounding_box = {
                let bounding_box = req.bounding_box.clone().unwrap();
                let min = bounding_box.min.unwrap();
                let max = bounding_box.max.unwrap();
                Aabb3::new(
                    Point3::new(min.x, min.y, min.z),
                    Point3::new(max.x, max.y, max.z),
                )
            };
            let mut reply = proto::GetPointsInBoxReply::new();

            // Computing the protobuf size is very expensive.
            // We compute the byte size of a Vector3f in the reply proto once outside the loop.
            let bytes_per_point = {
                let mut reply = proto::GetPointsInBoxReply::new();
                let initial_proto_size = reply.compute_size();
                let mut v = point_viewer::proto::Vector3f::new();
                v.set_x(1.);
                v.set_y(1.);
                v.set_z(1.);
                reply.mut_points().push(v);
                let final_proto_size = reply.compute_size();
                final_proto_size - initial_proto_size
            };
            // Proto message must be below 4 MB.
            let max_message_size = 4 * 1024 * 1024;
            let mut reply_size = 0;
            octree.points_in_box(&bounding_box).for_each(|p| {
                let mut v = point_viewer::proto::Vector3f::new();
                v.set_x(p.position.x);
                v.set_y(p.position.y);
                v.set_z(p.position.z);
                reply.mut_points().push(v);

                reply_size += bytes_per_point;
                if reply_size > max_message_size - bytes_per_point {
                    tx.send((reply.clone(), WriteFlags::default())).unwrap();
                    reply.mut_points().clear();
                    reply_size = 0;
                }
            });
        });

        // TODO(sirver): I did not figure out how to return meaningful errors. At least we return
        // any error.
        let rx = rx.map_err(|_| grpcio::Error::RemoteStopped);
        let f = resp
            .send_all(rx)
            .map(|_| {})
            .map_err(|e| println!("failed to reply: {:?}", e));
        ctx.spawn(f)
    }
}

fn main() {
    let matches = clap::App::new("octree_server")
        .args(&[
            clap::Arg::with_name("port")
                .help("Port to listen on for connections. [50051]")
                .long("port")
                .takes_value(true),
            clap::Arg::with_name("octree_directory")
                .help("Input directory of the octree directory to serve.")
                .index(1)
                .required(true),
        ])
        .get_matches();

    let port = value_t!(matches, "port", u16).unwrap_or(50051);
    let octree_directory = PathBuf::from(matches.value_of("octree_directory").unwrap());

    let env = Arc::new(Environment::new(1));
    let meta = read_meta_proto(&octree_directory).unwrap();
    let octree = Arc::new(OnDiskOctree::new(octree_directory).unwrap());

    let service = proto_grpc::create_octree(OctreeService { octree, meta });
    let mut server = ServerBuilder::new(env)
        .register_service(service)
        .bind("0.0.0.0" /* ip to bind to */, port)
        .build()
        .unwrap();
    server.start();
    for &(ref host, port) in server.bind_addrs() {
        println!("listening on {}:{}", host, port);
    }
    let (tx, rx) = oneshot::channel();
    thread::spawn(move || {
        println!("Press ENTER to exit...");
        let _ = io::stdin().read(&mut [0]).unwrap();
        tx.send(())
    });
    let _ = rx.wait();
    let _ = server.shutdown().wait();
}
