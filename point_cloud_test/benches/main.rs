use criterion::{black_box, criterion_group, criterion_main, Criterion};
use point_cloud_client::PointCloudClient;
use point_cloud_test_lib::{
    get_abb_query, get_cell_union_query, get_frustum_query, get_obb_query, make_octree,
    make_s2_cells, setup_octree_client, setup_s2_client, Arguments, SyntheticData,
};
use point_viewer::iterator::{PointLocation, PointQuery};
use tempdir::TempDir;

fn bench_octree_building_multithreaded(c: &mut Criterion) {
    let mut args = Arguments::default();
    args.num_points = 100_000;
    c.bench_function("bench_octree_building_multithreaded", |b| {
        b.iter(|| {
            let temp_dir = TempDir::new("octree").unwrap();
            make_octree(&args, temp_dir.path());
        })
    });
}

fn bench_s2_building_singlethreaded(c: &mut Criterion) {
    let mut args = Arguments::default();
    args.num_points = 100_000;
    c.bench_function("bench_s2_building_singlethreaded", |b| {
        b.iter(|| {
            let temp_dir = TempDir::new("s2").unwrap();
            make_s2_cells(&args, temp_dir.path());
        })
    });
}

fn all_query_octree(b: &mut Criterion) {
    run_bench(
        "all_query_octree",
        setup_octree_client,
        |_| PointLocation::AllPoints,
        b,
    )
}

fn all_query_s2(b: &mut Criterion) {
    run_bench(
        "all_query_s2",
        setup_s2_client,
        |_| PointLocation::AllPoints,
        b,
    )
}

fn box_query_octree(b: &mut Criterion) {
    run_bench("box_query_octree", setup_octree_client, get_abb_query, b)
}

fn box_query_s2(b: &mut Criterion) {
    run_bench("box_query_s2", setup_s2_client, get_abb_query, b)
}

fn frustum_query_octree(b: &mut Criterion) {
    run_bench(
        "frustum_query_octree",
        setup_octree_client,
        get_frustum_query,
        b,
    )
}

fn frustum_query_s2(b: &mut Criterion) {
    run_bench("frustum_query_s2", setup_s2_client, get_frustum_query, b)
}

fn obb_query_octree(b: &mut Criterion) {
    run_bench("obb_query_octree", setup_octree_client, get_obb_query, b)
}

fn obb_query_s2(b: &mut Criterion) {
    run_bench("obb_query_s2", setup_s2_client, get_obb_query, b)
}

fn cell_union_query_octree(b: &mut Criterion) {
    run_bench(
        "cell_union_query_octree",
        setup_octree_client,
        get_cell_union_query,
        b,
    )
}

fn cell_union_query_s2(b: &mut Criterion) {
    run_bench(
        "cell_union_query_s2",
        setup_s2_client,
        get_cell_union_query,
        b,
    )
}

criterion_group!(
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
criterion_main!(benches);

type SetupClientFn = fn(&Arguments) -> (PointCloudClient, SyntheticData);

fn run_bench<F>(name: &'static str, gen_client: SetupClientFn, gen_location: F, c: &mut Criterion)
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
    c.bench_function(name, |b| {
        b.iter(|| {
            let res = client.for_each_point_data(&query, |batch| {
                black_box(batch);
                Ok(())
            });
            assert!(res.is_ok());
        })
    });
}
