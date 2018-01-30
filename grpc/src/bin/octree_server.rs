extern crate cgmath;
#[macro_use]
extern crate clap;
extern crate futures;
extern crate grpcio;
extern crate point_viewer;
extern crate point_viewer_grpc;
extern crate protobuf;

use cgmath::Matrix4;
use futures::Future;
use futures::sync::oneshot;
use grpcio::{Environment, RpcContext, ServerBuilder, UnarySink};
use point_viewer::math::Cube;
use point_viewer::octree;
use point_viewer::octree::{NodeId, Octree, OnDiskOctree};
use point_viewer_grpc::proto;
use point_viewer_grpc::proto_grpc;
use std::{io, thread};
use std::io::Read;
use std::path::PathBuf;
use std::sync::Arc;

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
        let bounding_cube = Cube::bounding(&self.octree.bounding_box);
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
        resp.mut_node()
            .mut_bounding_cube()
            .mut_min()
            .set_x(data.meta.bounding_cube.min().x);
        resp.mut_node()
            .mut_bounding_cube()
            .mut_min()
            .set_y(data.meta.bounding_cube.min().y);
        resp.mut_node()
            .mut_bounding_cube()
            .mut_min()
            .set_z(data.meta.bounding_cube.min().z);
        resp.mut_node()
            .mut_bounding_cube()
            .set_edge_length(data.meta.bounding_cube.edge_length());
        resp.mut_node().set_num_points(data.meta.num_points);
        resp.set_position(data.position);
        resp.set_color(data.color);
        let f = sink.success(resp)
            .map_err(move |e| println!("failed to reply {:?}: {:?}", req, e));
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
