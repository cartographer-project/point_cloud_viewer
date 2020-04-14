use nalgebra::{Point3, Vector3};
use num_integer::div_ceil;
use point_cloud_test_lib::queries::*;
use point_cloud_test_lib::{setup_pointcloud, Arguments, SyntheticData};
use point_viewer::iterator::PointCloud;
use point_viewer::iterator::{PointLocation, PointQuery};
use point_viewer::math::{sat, ConvexPolyhedron, PointCulling};
use std::cmp::Ordering;

#[test]
fn num_points_in_octree_meta() {
    let args = Arguments::default();
    let (_, oct, _) = setup_pointcloud(&args);
    let meta = oct.to_meta_proto();
    assert!(meta.has_octree());
    let num_points: i64 = meta
        .get_octree()
        .get_nodes()
        .iter()
        .map(|n| n.num_points)
        .sum();
    assert_eq!(num_points, Arguments::default().num_points as i64);
}

#[test]
fn num_points_in_s2_meta() {
    let args = Arguments::default();
    let (s2, _, _) = setup_pointcloud(&args);
    let meta = s2.to_meta_proto();
    assert!(meta.has_s2());
    let num_points: u64 = meta.get_s2().get_cells().iter().map(|c| c.num_points).sum();
    assert_eq!(num_points, Arguments::default().num_points as u64);
}

#[test]
fn check_all_query_equality() {
    check_equality(|_| PointLocation::AllPoints)
}

#[test]
fn check_box_query_equality() {
    check_equality(get_aabb_query)
}

#[test]
fn check_frustum_query_equality() {
    check_equality(get_frustum_query)
}

#[test]
fn check_obb_query_equality() {
    check_equality(get_obb_query);
}

#[test]
fn check_cell_union_query_equality() {
    check_equality(get_cell_union_query)
}

#[test]
fn check_box_point_culling_equality() {
    check_point_culling_equality(get_aabb)
}

#[test]
fn check_frustum_point_culling_equality() {
    check_point_culling_equality(get_frustum)
}

#[test]
fn check_obb_point_culling_equality() {
    check_point_culling_equality(get_obb);
}

fn check_equality<F>(gen_location: F)
where
    F: FnOnce(SyntheticData) -> PointLocation,
{
    let args = Arguments::default();
    let (s2, oct, data) = setup_pointcloud(&args);
    let query = PointQuery {
        attributes: vec!["color"],
        location: gen_location(data),
        ..Default::default()
    };
    let points_oct = query_and_sort(&oct, &query, args.batch_size);
    let points_s2 = query_and_sort(&s2, &query, args.batch_size);
    assert_points_equal(&points_s2, &points_oct, args.resolution);
}

/// We have two implementations of a containment predicate on points:
/// One via the `PointCulling` trait, and one via the `ConvexPolyhedron` trait.
/// They should behave the same.
fn check_point_culling_equality<F, C>(gen_culling: F)
where
    F: FnOnce(SyntheticData) -> C,
    C: PointCulling<f64> + ConvexPolyhedron<f64>,
{
    let args = Arguments::default();
    let data = SyntheticData::new(args.width, args.height, args.num_points, args.seed);
    let c = gen_culling(data.clone());
    let intersector = c.intersector();
    let data_filtered = data.filter(|p| {
        let point = Point3::new(p.position.x, p.position.y, p.position.z);
        let sat_result = sat::sat(
            intersector.face_normals.iter().copied(),
            &intersector.corners,
            &[point],
        ) == sat::Relation::In;
        assert!(c.contains(&point) == sat_result);
        sat_result
    });
    assert!(
        data_filtered.count() > 0,
        "The query returned no points (using PointCulling only)"
    );
}

fn query_and_sort<C>(point_cloud: &C, query: &PointQuery, batch_size: usize) -> Vec<IndexedPoint>
where
    C: PointCloud,
{
    let mut points = Vec::new();
    for node_id in point_cloud.nodes_in_location(&query.location).into_iter() {
        point_cloud
            .stream_points_for_query_in_node(query, node_id, batch_size, |batch| {
                let color: &Vec<Vector3<u8>> = batch.get_attribute_vec("color")?;
                let indexed_point_iter = color.iter().zip(batch.position.iter()).map(|(c, p)| {
                    // Decode the index we encoded in the color
                    let idx = ((c.x as usize) << 16) + ((c.y as usize) << 8) + c.z as usize;
                    IndexedPoint { idx, pos: *p }
                });
                points.extend(indexed_point_iter);
                Ok(())
            })
            .unwrap();
    }
    points.sort_unstable_by(|p1, p2| p1.idx.cmp(&p2.idx));
    assert!(
        !points.is_empty(),
        "The query returned no points (using streaming)"
    );
    points
}

struct IndexedPoint {
    idx: usize,
    pos: Point3<f64>,
}

// Checks that the points are equal up to a precision of the default resolution
fn assert_points_equal(points_s2: &[IndexedPoint], points_oct: &[IndexedPoint], resolution: f64) {
    let mut num_skipped = 0;
    // If a point is allowed to be displaced by up to args.resolution in each dimension,
    // the distance from the true position may be up to 2 * resolution * sqrt(3).
    // We take 2 * resolution, since we can get errors when encoding and decoding.
    let threshold = 3.0_f64.sqrt() * 2.0 * resolution;
    let mut s2_iter = points_s2.iter().enumerate();
    let mut oct_iter = points_oct.iter().enumerate();
    let mut s2_next = s2_iter.next();
    let mut oct_next = oct_iter.next();

    while let (Some((count_s2, p_s2)), Some((count_oct, p_oct))) = (s2_next, oct_next) {
        match p_s2.idx.cmp(&p_oct.idx) {
            Ordering::Equal => {
                let distance = (p_s2.pos - p_oct.pos).norm();
                assert!(
                    distance <= threshold,
                    "Inequality between s2 point [{}]: {:?} and octree point [{}]: {:?}, distance {}",
                    count_s2,
                    p_s2.pos,
                    count_oct,
                    p_oct.pos,
                    distance
                );
                s2_next = s2_iter.next();
                oct_next = oct_iter.next();
            }
            Ordering::Less => {
                num_skipped += 1;
                s2_next = s2_iter.next();
            }
            Ordering::Greater => {
                num_skipped += 1;
                oct_next = oct_iter.next();
            }
        }
    }
    assert!(
        num_skipped <= div_ceil(std::cmp::min(points_s2.len(), points_oct.len()), 100),
        "More than 1% point index mismatches."
    );
}
