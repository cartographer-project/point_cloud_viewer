use crate::backend_error::PointsViewerError;
use crate::state::AppState;
use actix_web::{
    http::ContentEncoding, AsyncResponder, FutureResponse, HttpResponse, Json, Path, Query, State,
};
use base64::decode;
use byteorder::{LittleEndian, WriteBytesExt};
use cgmath::Matrix4;
use futures::future::{self, result, Future};
use point_viewer::octree::{self, Octree};
use std::str::FromStr;
use std::sync::Arc;
use time;

#[derive(Deserialize)]
pub struct Info {
    matrix: String,
}

/// Method that returns visible nodes
pub fn get_visible_nodes(
    (base64_octree_id, state, matrix_query): (Path<Vec<u8>>, State<Arc<AppState>>, Query<Info>),
) -> HttpResponse {
    println!("current tree {:?}", base64_octree_id);
    match get_octree_from_state(&base64_octree_id.into_inner(), &state) {
        Err(err) => HttpResponse::from_error(err.into()),
        Ok(octree) => {
            let matrix = {
                // Entries are column major.
                let e: Vec<f64> = matrix_query
                    .matrix
                    .split(',')
                    .map(|s| s.parse::<f64>().unwrap())
                    .collect();
                // matrix size check
                if 16 == e.len() {
                    Matrix4::new(
                        e[0], e[1], e[2], e[3], e[4], e[5], e[6], e[7], e[8], e[9], e[10], e[11],
                        e[12], e[13], e[14], e[15],
                    )
                } else {
                    return HttpResponse::from_error(
                        PointsViewerError::BadRequest(
                            "Parsing Error: Expected matrix with 16 elements".to_string(),
                        )
                        .into(),
                    );
                }
            };

            let visible_nodes = { octree.get_visible_nodes(&matrix) };
            let mut reply = String::from("[");
            let visible_nodes_string = visible_nodes
                .iter()
                .map(|id| format!("\"{}\"", id))
                .collect::<Vec<_>>()
                .join(",");
            reply.push_str(&visible_nodes_string);
            reply.push(']');

            HttpResponse::Ok()
                .content_type("application/json")
                .body(reply)
        }
    }
}

// Javascript requires its arrays to be padded to 8 bytes.
fn pad(input: &mut Vec<u8>) {
    let pad = input.len() % 8;
    if pad == 0 {
        return;
    }
    for _ in 0..(8 - pad) {
        input.push(0);
    }
}

fn get_octree_from_state(
    base64_octree_id: &[u8],
    state: &State<Arc<AppState>>,
) -> Result<Arc<Octree>, PointsViewerError> {
    // decode octree id
    let octree_id: String = match decode(base64_octree_id) {
        Ok(decoded) => String::from_utf8(decoded).unwrap(),
        Err(err) => {
            return Err(PointsViewerError::BadRequest(format!(
                "Base64 error: {:?}",
                err
            )));
        }
    };
    println!("current tree {}", octree_id);
    state.load_octree(&octree_id).map_err(|_error| {
        crate::backend_error::PointsViewerError::NotFound(format!(
            "Could not load tree with octree_id {}.",
            octree_id
        ))
    })
}

/// Asynchronous Handler to get Node Data
/// returns an alias for Box<Future<Item=HttpResponse, Error=Error>>;
pub fn get_nodes_data(
    (base64_octree_id, state, nodes): (Path<Vec<u8>>, State<Arc<AppState>>, Json<Vec<String>>),
) -> FutureResponse<HttpResponse> {
    let mut start = 0;
    future::ok(())
        .and_then(move |_| {
            start = time::precise_time_ns();
            let data: Vec<String> = Json::into_inner(nodes);
            let nodes_to_load = data
                .into_iter()
                .map(|e| octree::NodeId::from_str(e.as_str()).unwrap());

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
            let octree: Arc<octree::Octree> =
                get_octree_from_state(&base64_octree_id.into_inner(), &state).unwrap();
            for node_id in nodes_to_load {
                let mut node_data = octree
                    .get_node_data(&node_id)
                    .map_err(|_error| {
                        crate::backend_error::PointsViewerError::NotFound(format!(
                            "Could not get node {}.",
                            node_id
                        ))
                    })
                    .unwrap();

                // Write the bounding box information.
                let min = node_data.meta.bounding_cube.min();
                reply_blob.write_f64::<LittleEndian>(min.x).unwrap();
                reply_blob.write_f64::<LittleEndian>(min.y).unwrap();
                reply_blob.write_f64::<LittleEndian>(min.z).unwrap();
                reply_blob
                    .write_f64::<LittleEndian>(node_data.meta.bounding_cube.edge_length())
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

            let duration_ms = (time::precise_time_ns() - start) as f32 / 1_000_000.;
            println!(
                "Got {} nodes with {} points ({}ms).",
                num_nodes_fetched, num_points, duration_ms
            );

            result(Ok(HttpResponse::Ok()
                .content_type("application/octet-stream")
                // disabling default encoding:
                // Local test (same machine) default encoding doubles the computing time in that condition by saving only 10% of the data volume
                // TODO(catevita) tests are required to find the most meaningful option
                .content_encoding(ContentEncoding::Identity)
                .body(reply_blob)))
        })
        .responder() // this method AsyncResponder::responder() constructs a boxed Future
}
