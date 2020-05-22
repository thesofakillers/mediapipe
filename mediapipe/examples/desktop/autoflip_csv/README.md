### Steps to run the AutoFlip video cropping graph

1. See [the custom installation instructions](https://github.com/thesofakillers/mediapipe/blob/master/mediapipe/docs/install_autoflip.md) to set up MediaPipe.

2. Build the run_autoflip binary to process a local video.

   ```bash
   bazel build -c opt --copt -DMESA_EGL_NO_X11_HEADERS \
     mediapipe/examples/desktop/autoflip_csv:run_autoflip
   ```

3. Run AutoFlip

   ```bash
   GLOG_logtostderr=1 bazel-bin/mediapipe/examples/desktop/autoflip_csv/run_autoflip \
   --calculator_graph_config_file=mediapipe/examples/desktop/autoflip_csv/autoflip_graph.pbtxt \
   --input_side_packets=input_video_path=path/to/input.mp4,output_file_path=path/to/output.csv,aspect_ratio=w:h
   ```

4. View the cropped video.
