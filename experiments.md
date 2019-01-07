# master (2e5ebf8acc8b2ce6d276ec01baa6b11a4477170f) with benchmark debug output changes:

Creating the octree:

~~~bash
rm -rf ~/Downloads/v2/master/octree_just_colors; \
  mkdir -p ~/Downloads/v2/master/octree_just_colors; \
  gtime -v master_binaries/build_octree ~/Downloads/pointcloud.ply \
    --output_directory ~/Downloads/v2/master/octree_just_colors
~~~

Best run:
User time (seconds): 1809.84
System time (seconds): 688.82
Percent of CPU this job got: 392%
Elapsed (wall clock) time (h:mm:ss or m:ss): 10:36.72
Average shared text size (kbytes): 0
Average unshared data size (kbytes): 0
Average stack size (kbytes): 0
Average total size (kbytes): 0
Maximum resident set size (kbytes): 46072
Average resident set size (kbytes): 0
Major (requiring I/O) page faults: 34
Minor (reclaiming a frame) page faults: 1593051
Voluntary context switches: 181745
Involuntary context switches: 18850190
Swaps: 0
File system inputs: 0
File system outputs: 0
Socket messages sent: 0
Socket messages received: 0
Signals delivered: 0
Page size (bytes): 4096
Exit status: 0


Querying benchmark (gRPC):

~~~bash
gtime -v master_binaries/octree_benchmark ~/Downloads/v2/master/octree_just_colors --num-points 30000000
~~~
Best run:
RunningStats {
    count: 30000000,
    x_stats: 1565.4654913369 +/- 27.1493959220,
    y_stats: 1858.5378599631 +/- 20.7612140479,
    z_stats: -23.5902155921 +/- 1.4666555392
}
User time (seconds): 17.58
System time (seconds): 3.89
Percent of CPU this job got: 230%
Elapsed (wall clock) time (h:mm:ss or m:ss): 0:09.31
Average shared text size (kbytes): 0
Average unshared data size (kbytes): 0
Average stack size (kbytes): 0
Average total size (kbytes): 0
Maximum resident set size (kbytes): 187096
Average resident set size (kbytes): 0
Major (requiring I/O) page faults: 0
Minor (reclaiming a frame) page faults: 955796
Voluntary context switches: 10
Involuntary context switches: 20462
Swaps: 0
File system inputs: 0
File system outputs: 0
Socket messages sent: 1431
Socket messages received: 4798
Signals delivered: 0
Page size (bytes): 4096
Exit status: 0

Querying benchmark (no gRPC):
~~~bash
gtime -v master_binaries/octree_benchmark ~/Downloads/v2/master/octree_just_colors --num-points 30000000 --no-client
~~~
Running Stats:
RunningStats {
    count: 30000000,
    x_stats: 1565.4654913369 +/- 27.1493959220,
    y_stats: 1858.5378599631 +/- 20.7612140479,
    z_stats: -23.5902155921 +/- 1.4666555392
}
User time (seconds): 2.18
System time (seconds): 0.13
Percent of CPU this job got: 99%
Elapsed (wall clock) time (h:mm:ss or m:ss): 0:02.32
Average shared text size (kbytes): 0
Average unshared data size (kbytes): 0
Average stack size (kbytes): 0
Average total size (kbytes): 0
Maximum resident set size (kbytes): 11780
Average resident set size (kbytes): 0
Major (requiring I/O) page faults: 0
Minor (reclaiming a frame) page faults: 3108
Voluntary context switches: 0
Involuntary context switches: 482
Swaps: 0
File system inputs: 0
File system outputs: 0
Socket messages sent: 0
Socket messages received: 0
Signals delivered: 0
Page size (bytes): 4096
Exit status: 0

# v2

Creating the octree:

~~~bash
rm -rf ~/Downloads/v2/v2/octree_just_colors; \
  gtime -v target/release/build_octree ~/Downloads/pointcloud.ply \
    --output_directory ~/Downloads/v2/v2/octree_just_colors
~~~

Best run:
User time (seconds): 2514.12
System time (seconds): 650.59
Percent of CPU this job got: 407%
Elapsed (wall clock) time (h:mm:ss or m:ss): 12:56.89
Average shared text size (kbytes): 0
Average unshared data size (kbytes): 0
Average stack size (kbytes): 0
Average total size (kbytes): 0
Maximum resident set size (kbytes): 144348
Average resident set size (kbytes): 0
Major (requiring I/O) page faults: 0
Minor (reclaiming a frame) page faults: 32552798
Voluntary context switches: 88560
Involuntary context switches: 6248375
Swaps: 0
File system inputs: 0
File system outputs: 0
Socket messages sent: 0
Socket messages received: 0
Signals delivered: 0
Page size (bytes): 4096
Exit status: 0
