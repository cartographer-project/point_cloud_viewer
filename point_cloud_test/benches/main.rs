// TODO(feuerste): Use the #test framework instead of bencher once stabilized.
use bencher::{benchmark_group, benchmark_main, black_box, Bencher};
use point_cloud_client::PointCloudClient;
use point_cloud_test_lib::{
    get_abb_query, get_cell_union_query, get_frustum_query, get_obb_query, make_octree,
    make_s2_cells, setup_octree_client, setup_s2_client, Arguments, SyntheticData,
};
use point_viewer::iterator::{PointLocation, PointQuery};
use tempdir::TempDir;

fn bench_octree_building_multithreaded(b: &mut Bencher) {
    let mut args = Arguments::default();
    args.num_points = 100_000;
    b.iter(|| {
        let temp_dir = TempDir::new("octree").unwrap();
        make_octree(&args, temp_dir.path());
    });
}

fn bench_s2_building_singlethreaded(b: &mut Bencher) {
    let mut args = Arguments::default();
    args.num_points = 100_000;
    b.iter(|| {
        let temp_dir = TempDir::new("s2").unwrap();
        make_s2_cells(&args, temp_dir.path());
    });
}

fn all_query_octree(b: &mut Bencher) {
    run_bench(setup_octree_client, |_| PointLocation::AllPoints, b)
}

fn all_query_s2(b: &mut Bencher) {
    run_bench(setup_s2_client, |_| PointLocation::AllPoints, b)
}

fn box_query_octree(b: &mut Bencher) {
    run_bench(setup_octree_client, get_abb_query, b)
}

fn box_query_s2(b: &mut Bencher) {
    run_bench(setup_s2_client, get_abb_query, b)
}

fn frustum_query_octree(b: &mut Bencher) {
    run_bench(setup_octree_client, get_frustum_query, b)
}

fn frustum_query_s2(b: &mut Bencher) {
    run_bench(setup_s2_client, get_frustum_query, b)
}

fn obb_query_octree(b: &mut Bencher) {
    run_bench(setup_octree_client, get_obb_query, b)
}

fn obb_query_s2(b: &mut Bencher) {
    run_bench(setup_s2_client, get_obb_query, b)
}

fn cell_union_query_octree(b: &mut Bencher) {
    run_bench(setup_octree_client, get_cell_union_query, b)
}

fn cell_union_query_s2(b: &mut Bencher) {
    run_bench(setup_s2_client, get_cell_union_query, b)
}

benchmark_group!(
    benches,
    bench_octree_building_multithreaded,
    bench_s2_building_singlethreaded,
    all_query_octree,
    all_query_s2,
    box_query_octree,
    box_query_s2,
    frustum_query_octree,
    frustum_query_s2,
    obb_query_octree,
    obb_query_s2,
    cell_union_query_octree,
    cell_union_query_s2,
);
benchmark_main!(benches);

type SetupClientFn = fn(&Arguments) -> (PointCloudClient, SyntheticData);

fn run_bench<F>(gen_client: SetupClientFn, gen_location: F, b: &mut Bencher)
where
    F: FnOnce(SyntheticData) -> PointLocation,
{
    let args = Arguments::default();
    let (client, data) = gen_client(&args);
    let query = PointQuery {
        attributes: vec!["color"],
        location: gen_location(data),
        ..Default::default()
    };
    b.iter(|| {
        let res = client.for_each_point_data(&query, |batch| {
            black_box(batch);
            Ok(())
        });
        assert!(res.is_ok());
    });
}
