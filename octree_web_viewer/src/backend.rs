use actix_web::{
    dev::Handler, http::ContentEncoding, AsyncResponder, Error, FromRequest, FutureResponse,
    HttpRequest, HttpResponse, Json, Result,
};
use byteorder::{LittleEndian, WriteBytesExt};
use cgmath::Matrix4;
use futures::future::{result, Future};
use point_viewer::octree::{self, Octree};
use std::sync::Arc;
use time;

pub struct VisibleNodes {
    octree: Arc<dyn Octree>,
}

impl VisibleNodes {
    pub fn new(octree: Arc<dyn Octree>) -> Self {
        VisibleNodes { octree }
    }
}

// CAVEAT from actix docs
//Be careful with synchronization primitives like Mutex or RwLock.
//The actix-web framework handles requests asynchronously.
//By blocking thread execution, all concurrent request handling processes would block.
//If you need to share or update some state from multiple threads, consider using the actix actor system.

impl<S> Handler<S> for VisibleNodes {
    type Result = Result<HttpResponse, Error>;

    fn handle(&self, req: &HttpRequest<S>) -> Self::Result {
        let matrix = {
            // Entries are column major.
            let e: Vec<f32> = req
                .query()
                .get("matrix")
                .expect("4x4 Matrix information expected")
                .split(',')
                .map(|s| s.parse::<f32>().unwrap())
                .collect();
            assert!(16 == e.len(), "4 by 4 Matrix required"); //matri
            Matrix4::new(
                e[0], e[1], e[2], e[3], e[4], e[5], e[6], e[7], e[8], e[9], e[10], e[11], e[12],
                e[13], e[14], e[15],
            )
        };
        //get the nodes from octree
        let visible_nodes = { self.octree.get_visible_nodes(&matrix) };
        let mut reply = String::from("[");
        let visible_nodes_string = visible_nodes
            .iter()
            .map(|id| format!("\"{}\"", id))
            .collect::<Vec<_>>()
            .join(",");
        reply.push_str(&visible_nodes_string);
        reply.push(']');
        //create response
        Ok(HttpResponse::Ok()
            .content_type("application/json")
            .body(reply))
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
    octree: Arc<dyn Octree>,
}

impl NodesData {
    pub fn new(octree: Arc<dyn Octree>) -> Self {
        NodesData { octree }
    }
}

impl<S: 'static> Handler<S> for NodesData {
    type Result = FutureResponse<HttpResponse>; //alias forBox<Future<Item=HttpResponse, Error=Error>>;

    fn handle(&self, req: &HttpRequest<S>) -> Self::Result {
        let octree = Arc::clone(&self.octree); //has to be moved into future
        let message_body_future = Json::<Vec<String>>::extract(req).from_err();
        message_body_future
            .and_then(move |extract_result| {
                let data: Vec<String> = Json::into_inner(extract_result);
                let nodes_to_load = data
                    .into_iter()
                    .map(|e| octree::NodeId::from_str(e.as_str()));

                // So this is godawful: We need to get data to the GPU without JavaScript herp-derping with
                // it - because that will stall interaction. The straight forward approach would be to ship
                // json with base64 encoded values - unfortunately base64 decoding in JavaScript yields a
                // string which cannot be used as a buffer. So we would need to manually convert this into
                // an Array with is very slow.
                // The alternative is to binary encode the whole request and parse it on the client side,
                // which requires careful constructing on the server and parsing on the client.
                let start = time::precise_time_ns(); //timing is not comparable to the previous implementation in iron
                let mut reply_blob = Vec::<u8>::new();

                let mut num_nodes_fetched = 0;
                let mut num_points = 0;
                for node_id in nodes_to_load {
                    let mut node_data = match octree.get_node_data(&node_id) {
                        Ok(n) => n,
                        Err(err) => panic!("Could not get node {}: {} ", node_id, err),
                    };

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
                    let bytes_per_coordinate =
                        node_data.meta.position_encoding.bytes_per_coordinate();
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
                } //end for

                let duration_ms = (time::precise_time_ns() - start) as f32 / 1000000.;
                println!(
                    "Got {} nodes with {} points ({}ms).",
                    num_nodes_fetched, num_points, duration_ms
                );

                result(Ok(HttpResponse::Ok()
                    .content_type("application/octet-stream")
                    .content_encoding(ContentEncoding::Identity) //2x speed as we are sending bytes, ca. 10% compression lost
                    .body(reply_blob)))
            }) // Construct boxed future by using `AsyncResponder::responder()` method
            .responder()
    }
}
