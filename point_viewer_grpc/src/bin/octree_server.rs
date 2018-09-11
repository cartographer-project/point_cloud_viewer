extern crate cgmath;
#[macro_use]
extern crate clap;
extern crate collision;
extern crate futures;
extern crate grpcio;
extern crate point_viewer;
extern crate point_viewer_grpc;
extern crate protobuf;

use cgmath::{Deg, Matrix4, PerspectiveFov, Point3, Rad, Transform, Vector3};
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

pub trait GetBytes {
    fn get_bytes_per_point(&mut self) -> u32;
}

impl GetBytes for proto::GetPointsInFrustumReply {
    fn get_bytes_per_point(&mut self) -> u32 {
        let initial_proto_size = self.compute_size();
        let mut v = point_viewer::proto::Vector3f::new();
        v.set_x(1.);
        v.set_y(1.);
        v.set_z(1.);
        self.mut_points().push(v);
        let final_proto_size = self.compute_size();
        self.mut_points().clear();
        final_proto_size - initial_proto_size
    }
}

impl GetBytes for proto::GetPointsInBoxReply {
    fn get_bytes_per_point(&mut self) -> u32 {
        let initial_proto_size = self.compute_size();
        let mut v = point_viewer::proto::Vector3f::new();
        v.set_x(1.);
        v.set_y(1.);
        v.set_z(1.);
        self.mut_points().push(v);
        let final_proto_size = self.compute_size();
        self.mut_points().clear();
        final_proto_size - initial_proto_size
    }
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

            let projection_matrix = Matrix4::from(PerspectiveFov {
                fovy: Rad::from(Deg((req.fov_y as f32).into())),
                aspect: (req.aspect as f32).into(),
                near: (req.z_near as f32).into(),
                far: (req.z_far as f32).into(),
            });
            let req_view_position = req.view_position.clone().unwrap();
            let req_view_direction = req.view_direction.clone().unwrap();
            let req_view_up = req.view_up.clone().unwrap();

            let view_position = Point3::new(req_view_position.x, req_view_position.y, req_view_position.z);
            // view_direction(not being used): unit vector defining the `direction` vector
            let _view_direction = Vector3::new(req_view_direction.x, req_view_direction.y, req_view_direction.z);
            // view_up: unit vector defining the `up` vector
            let view_up = Vector3::new(req_view_up.x, req_view_up.y, req_view_up.z);
            // view_center: a vector to look at `center` from `eye` using `up` for orientation.
            let view_center = view_position + view_up;

            let view_transform: Matrix4<f32> = Transform::look_at(view_position, view_center, view_up);
            let view_matrix: Matrix4<f32> = view_transform.inverse_transform().unwrap().into();
            let frustum_matrix = projection_matrix * view_matrix;

            let mut reply = proto::GetPointsInFrustumReply::new();

            let bytes_per_point = reply.get_bytes_per_point();

            // Proto message must be below 4 MB.
            let max_message_size = 4 * 1024 * 1024;
            let mut reply_size = 0;
            octree.points_in_frustum(&frustum_matrix).for_each(|p| {
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
            tx.send((reply, WriteFlags::default())).unwrap();
        });

        let rx = rx.map_err(|_| grpcio::Error::RemoteStopped);
        let f = resp
            .send_all(rx)
            .map(|_| {})
            .map_err(|e| println!("failed to reply: {:?}", e));
        ctx.spawn(f)
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
            let bytes_per_point = reply.get_bytes_per_point();
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
