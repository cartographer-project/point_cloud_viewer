extern crate cgmath;
#[macro_use]
extern crate clap;
extern crate collision;
extern crate futures;
extern crate grpcio;
extern crate point_viewer;
extern crate point_viewer_grpc;
extern crate protobuf;

use cgmath::{Matrix4, Point3};
use collision::Aabb3;
use futures::{stream, Future, Sink};
use futures::sync::oneshot;
use grpcio::{Environment, RpcContext, ServerBuilder, ServerStreamingSink, UnarySink, WriteFlags};
use point_viewer::InternalIterator;
use point_viewer::math::Cube;
use point_viewer::octree;
use point_viewer::octree::{NodeId, Octree, OnDiskOctree};
use point_viewer_grpc::proto;
use point_viewer_grpc::proto_grpc;
use std::{io, thread};
use std::io::Read;
use std::path::PathBuf;
use std::sync::Arc;
use protobuf::Message;

#[derive(Clone)]
struct OctreeService {
    octree: Arc<OnDiskOctree>,
}

impl proto_grpc::Octree for OctreeService {
    fn get_root_bounding_cube(
        &self,
        ctx: RpcContext,
        req: proto::GetRootBoundingCubeRequest,
        sink: UnarySink<proto::GetRootBoundingCubeReply>,
    ) {
        let bounding_cube = Cube::bounding(&self.octree.bounding_box());
        let mut resp = proto::GetRootBoundingCubeReply::new();
        resp.mut_bounding_cube()
            .mut_min()
            .set_x(bounding_cube.min().x);
        resp.mut_bounding_cube()
            .mut_min()
            .set_y(bounding_cube.min().y);
        resp.mut_bounding_cube()
            .mut_min()
            .set_z(bounding_cube.min().z);
        resp.mut_bounding_cube()
            .set_edge_length(bounding_cube.edge_length());
        let f = sink.success(resp)
            .map_err(move |e| println!("failed to reply {:?}: {:?}", req, e));
        ctx.spawn(f)
    }

    fn get_visible_nodes(
        &self,
        ctx: RpcContext,
        req: proto::GetVisibleNodesRequest,
        sink: UnarySink<proto::GetVisibleNodesReply>,
    ) {
        let projection_matrix = Matrix4::new(
            req.projection_matrix[0],
            req.projection_matrix[1],
            req.projection_matrix[2],
            req.projection_matrix[3],
            req.projection_matrix[4],
            req.projection_matrix[5],
            req.projection_matrix[6],
            req.projection_matrix[7],
            req.projection_matrix[8],
            req.projection_matrix[9],
            req.projection_matrix[10],
            req.projection_matrix[11],
            req.projection_matrix[12],
            req.projection_matrix[13],
            req.projection_matrix[14],
            req.projection_matrix[15],
        );
        let result = self.octree.get_visible_nodes(
            &projection_matrix,
            req.get_width(),
            req.get_height(),
            octree::UseLod::No,
        );

        let mut resp = proto::GetVisibleNodesReply::new();
        for node in result {
            resp.mut_node_ids().push(node.id.to_string());
        }
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
            .get_node_data(&NodeId::from_str(&req.id), 1)
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

    fn get_points_in_box(
        &self,
        ctx: RpcContext,
        req: proto::GetPointsInBoxRequest,
        resp: ServerStreamingSink<proto::GetPointsInBoxReply>,
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
        let mut replies = Vec::new();
        replies.push((proto::GetPointsInBoxReply::new(), WriteFlags::default()));
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
        self.octree.points_in_box(&bounding_box).for_each(|p| {
            let mut v = point_viewer::proto::Vector3f::new();
            v.set_x(p.position.x);
            v.set_y(p.position.y);
            v.set_z(p.position.z);
            replies.last_mut().unwrap().0.mut_points().push(v);
            reply_size += bytes_per_point;
            if reply_size > max_message_size - bytes_per_point {
                replies.push((proto::GetPointsInBoxReply::new(), WriteFlags::default()));
                reply_size = 0;
            }
        });
        let f = resp.send_all(stream::iter_ok::<_, grpcio::Error>(replies))
            .map(|_| {})
            .map_err(|e| println!("failed to rply: {:?}", e));
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
    let octree = Arc::new(OnDiskOctree::new(octree_directory).unwrap());

    let service = proto_grpc::create_octree(OctreeService { octree });
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
