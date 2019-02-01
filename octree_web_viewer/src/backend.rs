use byteorder::{LittleEndian, WriteBytesExt};
use cgmath::Matrix4;
use iron;
use iron::mime::Mime;
use iron::prelude::*;
use json;
use point_viewer::octree::{self, Octree};
use std::io::Read;
use std::sync::{Arc, RwLock};
use time;
use urlencoded::UrlEncodedQuery;

pub struct VisibleNodes {
    octree: Arc<RwLock<Octree>>,
}

impl VisibleNodes {
    pub fn new(octree: Arc<RwLock<Octree>>) -> Self {
        VisibleNodes { octree }
    }
}

impl iron::Handler for VisibleNodes {
    fn handle(&self, req: &mut Request) -> IronResult<Response> {
        // TODO(hrapp): This should not crash on error, but return a valid http response.
        let query = req.get_ref::<UrlEncodedQuery>().unwrap();
        let matrix = {
            // Entries are column major.
            let e: Vec<f32> = query.get("matrix").unwrap()[0]
                .split(',')
                .map(|s| s.parse::<f32>().unwrap())
                .collect();
            Matrix4::new(
                e[0], e[1], e[2], e[3], e[4], e[5], e[6], e[7], e[8], e[9], e[10], e[11], e[12],
                e[13], e[14], e[15],
            )
        };

        let visible_nodes = {
            let octree = self.octree.read().unwrap();
            octree.get_visible_nodes(&matrix)
        };
        let mut reply = String::from("[");
        let visible_nodes_string = visible_nodes
            .iter()
            .map(|id| format!("\"{}\"", id))
            .collect::<Vec<_>>()
            .join(",");
        reply.push_str(&visible_nodes_string);
        reply.push(']');
        let content_type = "application/json".parse::<Mime>().unwrap();
        Ok(Response::with((content_type, iron::status::Ok, reply)))
    }
}

// Javascript requires its arrays to be padded to 4 bytes.
fn pad(input: &mut Vec<u8>) {
    let pad = input.len() % 4;
    if pad == 0 {
        return;
    }
    for _ in 0..(4 - pad) {
        input.push(0);
    }
}

pub struct NodesData {
    octree: Arc<RwLock<Octree>>,
}

impl NodesData {
    pub fn new(octree: Arc<RwLock<Octree>>) -> Self {
        NodesData { octree }
    }
}

impl iron::Handler for NodesData {
    fn handle(&self, req: &mut Request) -> IronResult<Response> {
        let start = time::precise_time_ns();
        let mut content = String::new();
        // TODO(hrapp): This should not crash on error, but return a valid http response.
        req.body.read_to_string(&mut content).unwrap();
        let data = json::parse(&content).unwrap();
        let nodes_to_load = data
            .members()
            .map(|e| octree::NodeId::from_str(e.as_str().unwrap()));

        // So this is godawful: We need to get data to the GPU without JavaScript herp-derping with
        // it - because that will stall interaction. The straight forward approach would be to ship
        // json with base64 encoded values - unfortunately base64 decoding in JavaScript yields a
        // string which cannot be used as a buffer. So we would need to manually convert this into
        // an Array with is very slow.
        // The alternative is to binary encode the whole request and parse it on the client side,
        // which requires careful constructing on the server and parsing on the client.
        let mut reply_blob = Vec::<u8>::new();

        let mut num_nodes_fetched = 0;
        let mut num_points = 0;
        let octree = self.octree.read().unwrap();
        for node_id in nodes_to_load {
            let mut node_data = octree.get_node_data(&node_id).unwrap();

            // Write the bounding box information.
            let min = node_data.meta.bounding_cube.min();
            reply_blob.write_f32::<LittleEndian>(min.x).unwrap();
            reply_blob.write_f32::<LittleEndian>(min.y).unwrap();
            reply_blob.write_f32::<LittleEndian>(min.z).unwrap();
            reply_blob
                .write_f32::<LittleEndian>(node_data.meta.bounding_cube.edge_length())
                .unwrap();

            // Number of points.
            reply_blob
                .write_u32::<LittleEndian>(node_data.meta.num_points as u32)
                .unwrap();

            // Position encoding.
            let bytes_per_coordinate = node_data.meta.position_encoding.bytes_per_coordinate();
            reply_blob.write_u8(bytes_per_coordinate as u8).unwrap();
            assert!(
                bytes_per_coordinate * node_data.meta.num_points as usize * 3
                    == node_data.position.len()
            );
            assert!(node_data.meta.num_points as usize * 3 == node_data.color.len());
            pad(&mut reply_blob);

            reply_blob.append(&mut node_data.position);
            pad(&mut reply_blob);

            reply_blob.append(&mut node_data.color);
            pad(&mut reply_blob);

            num_nodes_fetched += 1;
            num_points += node_data.meta.num_points;
        }

        let duration_ms = (time::precise_time_ns() - start) as f32 / 1000000.;
        println!(
            "Got {} nodes with {} points ({}ms).",
            num_nodes_fetched, num_points, duration_ms
        );

        let content_type = "application/octet-stream".parse::<Mime>().unwrap();
        Ok(Response::with((content_type, iron::status::Ok, reply_blob)))
    }
}
