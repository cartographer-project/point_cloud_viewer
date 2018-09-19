extern crate cgmath;
#[macro_use]
extern crate clap;
extern crate collision;
extern crate futures;
extern crate grpcio;
extern crate point_viewer;
extern crate point_viewer_grpc;
extern crate protobuf;

use cgmath::{Decomposed, Matrix4, PerspectiveFov, Point3, Quaternion, Rad, Transform, Vector3};
use collision::Aabb3;
use futures::{Stream, Future, Sink};
use futures::sync::oneshot;
use grpcio::{Environment, RpcContext, ServerBuilder, ServerStreamingSink, UnarySink, WriteFlags};
use point_viewer::{InternalIterator, Point};
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

#[derive(Debug)]
enum OctreeQuery {
    FrustumQuery(Matrix4<f32>),
    BoxQuery(Aabb3<f32>),
}

fn stream_points_back_to_sink(
    query: OctreeQuery,
    octree: Arc<OnDiskOctree>,
    ctx: RpcContext,
    resp: ServerStreamingSink<proto::PointsReply>)
{
    use thread;

    // This creates a async-aware (tx, rx) pair that can wake up the event loop when new data
    // is piped through it.
    let (tx, rx) = mpsc::channel(4);
    thread::spawn(move || {
        // This is the secret sauce connecting an OS thread to a event-based receiver. Calling
        // wait() on this turns the event aware, i.e. async 'tx' into a blocking 'tx' that will
        // make this thread block when the event loop is not quick enough with piping out data.
        let mut tx = tx.wait();

        let mut reply = proto::PointsReply::new();
        let bytes_per_point = { 
            let initial_proto_size = reply.compute_size();
            let mut v = point_viewer::proto::Vector3f::new();
            v.set_x(1.);
            v.set_y(1.);
            v.set_z(1.);
            reply.mut_points().push(v);
            let final_proto_size = reply.compute_size();
            reply.mut_points().clear();
            final_proto_size - initial_proto_size
        };

        // Proto message must be below 4 MB.
        let max_message_size = 4 * 1024 * 1024;
        let mut reply_size = 0;

        {
            // Extra scope to make sure that 'func' does not outlive 'reply'.
            let func = |p: &Point| {
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
            };
            match query {
                OctreeQuery::BoxQuery(bounding_box) => octree.points_in_box(&bounding_box).for_each(func),
                OctreeQuery::FrustumQuery(frustum_matrix) => octree.points_in_frustum(&frustum_matrix).for_each(func),
            };
        }
        tx.send((reply, WriteFlags::default())).unwrap();
    });

    let rx = rx.map_err(|_| grpcio::Error::RemoteStopped);
    let f = resp
        .send_all(rx)
        .map(|_| {})
        .map_err(|e| println!("failed to reply: {:?}", e));
    ctx.spawn(f)
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
        resp: ServerStreamingSink<proto::PointsReply>,
    ) {
        let projection_matrix = Matrix4::from(PerspectiveFov {
            fovy: Rad(req.fovy_rad as f32),
            aspect: (req.aspect as f32).into(),
            near: (req.z_near as f32).into(),
            far: (req.z_far as f32).into(),
        });
        let rotation = {
            let q = req.rotation.unwrap();
            Quaternion::new( q.w, q.x, q.y, q.z)
        };

        let translation = {
            let t = req.translation.unwrap();
            Vector3::new(t.x, t.y, t.z)
        };

        let view_transform = Decomposed{
            scale: 1.0,
            rot: rotation,
            disp: translation,
        };
        let frustum_matrix = projection_matrix.concat(&view_transform.into());
        stream_points_back_to_sink(OctreeQuery::FrustumQuery(frustum_matrix), self.octree.clone(), ctx, resp)
    }

    fn get_points_in_box(
        &self,
        ctx: RpcContext,
        req: proto::GetPointsInBoxRequest,
        resp: ServerStreamingSink<proto::PointsReply>,
        ) {
        let bounding_box = {
            let bounding_box = req.bounding_box.clone().unwrap();
            let min = bounding_box.min.unwrap();
            let max = bounding_box.max.unwrap();
            Aabb3::new(
                Point3::new(min.x, min.y, min.z),
                Point3::new(max.x, max.y, max.z),
                )
        };
        stream_points_back_to_sink(OctreeQuery::BoxQuery(bounding_box), self.octree.clone(), ctx, resp)
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
