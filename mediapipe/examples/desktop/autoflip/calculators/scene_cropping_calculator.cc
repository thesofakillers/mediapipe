// Copyright 2019 The MediaPipe Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mediapipe/examples/desktop/autoflip/calculators/scene_cropping_calculator.h"

#include <cmath>

#include "absl/memory/memory.h"
#include "absl/strings/str_format.h"
#include "mediapipe/examples/desktop/autoflip/autoflip_messages.pb.h"
#include "mediapipe/examples/desktop/autoflip/quality/scene_cropping_viz.h"
#include "mediapipe/examples/desktop/autoflip/quality/utils.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/port/opencv_core_inc.h"
#include "mediapipe/framework/port/opencv_imgproc_inc.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/ret_check.h"
#include "mediapipe/framework/port/status.h"
#include "mediapipe/framework/timestamp.h"

namespace mediapipe {
namespace autoflip {

constexpr char kInputVideoFrames[] = "VIDEO_FRAMES";
constexpr char kInputKeyFrames[] = "KEY_FRAMES";
constexpr char kInputDetections[] = "DETECTION_FEATURES";
constexpr char kInputStaticFeatures[] = "STATIC_FEATURES";
constexpr char kInputShotBoundaries[] = "SHOT_BOUNDARIES";
constexpr char kInputExternalSettings[] = "EXTERNAL_SETTINGS";
// This side packet must be used in conjunction with
// TargetSizeType::MAXIMIZE_TARGET_DIMENSION
constexpr char kAspectRatio[] = "EXTERNAL_ASPECT_RATIO";

constexpr char kOutputCroppedFrames[] = "CROPPED_FRAMES";
constexpr char kOutputKeyFrameCropViz[] = "KEY_FRAME_CROP_REGION_VIZ_FRAMES";
constexpr char kOutputFocusPointFrameViz[] = "SALIENT_POINT_FRAME_VIZ_FRAMES";
constexpr char kOutputSummary[] = "CROPPING_SUMMARY";

::mediapipe::Status SceneCroppingCalculator::GetContract(
    ::mediapipe::CalculatorContract* cc) {
  if (cc->InputSidePackets().HasTag(kInputExternalSettings)) {
    cc->InputSidePackets().Tag(kInputExternalSettings).Set<std::string>();
  }
  if (cc->InputSidePackets().HasTag(kAspectRatio)) {
    cc->InputSidePackets().Tag(kAspectRatio).Set<std::string>();
  }
  RET_CHECK(cc->InputSidePackets().HasTag("OUTPUT_FILE_PATH"));
  cc->InputSidePackets().Tag("OUTPUT_FILE_PATH").Set<std::string>();
  cc->Inputs().Tag(kInputVideoFrames).Set<ImageFrame>();
  if (cc->Inputs().HasTag(kInputKeyFrames)) {
    cc->Inputs().Tag(kInputKeyFrames).Set<ImageFrame>();
  }
  cc->Inputs().Tag(kInputDetections).Set<DetectionSet>();
  if (cc->Inputs().HasTag(kInputStaticFeatures)) {
    cc->Inputs().Tag(kInputStaticFeatures).Set<StaticFeatures>();
  }
  cc->Inputs().Tag(kInputShotBoundaries).Set<bool>();

  return ::mediapipe::OkStatus();
}

::mediapipe::Status SceneCroppingCalculator::Open(CalculatorContext* cc) {
  options_ = cc->Options<SceneCroppingCalculatorOptions>();
  RET_CHECK_GT(options_.max_scene_size(), 0)
      << "Maximum scene size is non-positive.";
  RET_CHECK_GE(options_.prior_frame_buffer_size(), 0)
      << "Prior frame buffer size is negative.";

  RET_CHECK(options_.solid_background_frames_padding_fraction() >= 0.0 &&
            options_.solid_background_frames_padding_fraction() <= 1.0)
      << "Solid background frames padding fraction is not in [0, 1].";
  const auto& padding_params = options_.padding_parameters();
  background_contrast_ = padding_params.background_contrast();
  RET_CHECK(background_contrast_ >= 0.0 && background_contrast_ <= 1.0)
      << "Background contrast " << background_contrast_ << " is not in [0, 1].";
  blur_cv_size_ = padding_params.blur_cv_size();
  RET_CHECK_GT(blur_cv_size_, 0) << "Blur cv size is non-positive.";
  overlay_opacity_ = padding_params.overlay_opacity();
  RET_CHECK(overlay_opacity_ >= 0.0 && overlay_opacity_ <= 1.0)
      << "Overlay opacity " << overlay_opacity_ << " is not in [0, 1].";

  scene_cropper_ = absl::make_unique<SceneCropper>();
  if (cc->Outputs().HasTag(kOutputSummary)) {
    summary_ = absl::make_unique<VideoCroppingSummary>();
  }
  // handles output file details, removing it if it already exists
  output_file_path_ =
      cc->InputSidePackets().Tag("OUTPUT_FILE_PATH").Get<std::string>();
  remove(output_file_path_.c_str());
  return ::mediapipe::OkStatus();
}

namespace {
::mediapipe::Status ParseAspectRatioString(
    const std::string& aspect_ratio_string, double* aspect_ratio) {
  std::string error_msg =
      "Aspect ratio std::string must be in the format of 'width:height', e.g. "
      "'1:1' or '5:4', your input was " +
      aspect_ratio_string;
  auto pos = aspect_ratio_string.find(":");
  RET_CHECK(pos != std::string::npos) << error_msg;
  double width_ratio;
  RET_CHECK(absl::SimpleAtod(aspect_ratio_string.substr(0, pos), &width_ratio))
      << error_msg;
  double height_ratio;
  RET_CHECK(absl::SimpleAtod(
      aspect_ratio_string.substr(pos + 1, aspect_ratio_string.size()),
      &height_ratio))
      << error_msg;
  *aspect_ratio = width_ratio / height_ratio;
  return ::mediapipe::OkStatus();
}
}  // namespace

::mediapipe::Status SceneCroppingCalculator::Process(
    ::mediapipe::CalculatorContext* cc) {
  // Sets frame dimension and format.  
  if (frame_width_ < 0 &&
      !cc->Inputs().Tag(kInputVideoFrames).Value().IsEmpty()) {
    const auto& frame = cc->Inputs().Tag(kInputVideoFrames).Get<ImageFrame>();
    frame_width_ = frame.Width();
    RET_CHECK_GT(frame_width_, 0) << "Input frame width is non-positive.";
    frame_height_ = frame.Height();
    RET_CHECK_GT(frame_height_, 0) << "Input frame height is non-positive.";
    frame_format_ = frame.Format();
    target_width_ = options_.target_width();
    target_height_ = options_.target_height();
    if (cc->InputSidePackets().HasTag(kInputExternalSettings)) {
      auto conversion_options = ParseTextProtoOrDie<ConversionOptions>(
          cc->InputSidePackets()
              .Tag(kInputExternalSettings)
              .Get<std::string>());
      target_width_ = conversion_options.target_width();
      target_height_ = conversion_options.target_height();
    }
    target_aspect_ratio_ = static_cast<double>(target_width_) / target_height_;
    RET_CHECK_NE(options_.target_size_type(),
                 SceneCroppingCalculatorOptions::UNKNOWN)
        << "TargetSizeType not set properly.";
    // Resets target size if keep original height or width.
    if (options_.target_size_type() ==
        SceneCroppingCalculatorOptions::KEEP_ORIGINAL_HEIGHT) {
      target_height_ = frame_height_;
      target_width_ = std::round(target_height_ * target_aspect_ratio_);
    } else if (options_.target_size_type() ==
               SceneCroppingCalculatorOptions::KEEP_ORIGINAL_WIDTH) {
      target_width_ = frame_width_;
      target_height_ = std::round(target_width_ / target_aspect_ratio_);
    } else if (options_.target_size_type() ==
               SceneCroppingCalculatorOptions::MAXIMIZE_TARGET_DIMENSION) {
      RET_CHECK(cc->InputSidePackets().HasTag(kAspectRatio))
          << "MAXIMIZE_TARGET_DIMENSION is set without an "
             "external_aspect_ratio";
      double requested_aspect_ratio;
      MP_RETURN_IF_ERROR(ParseAspectRatioString(
          cc->InputSidePackets().Tag(kAspectRatio).Get<std::string>(),
          &requested_aspect_ratio));
      const double original_aspect_ratio =
          static_cast<double>(frame_width_) / frame_height_;
      if (original_aspect_ratio > requested_aspect_ratio) {
        target_height_ = frame_height_;
        target_width_ = std::round(target_height_ * requested_aspect_ratio);
      } else {
        target_width_ = frame_width_;
        target_height_ = std::round(target_width_ / requested_aspect_ratio);
      }
    }
    // Makes sure that target size is even if keep original width or height.
    if (options_.target_size_type() !=
        SceneCroppingCalculatorOptions::USE_TARGET_DIMENSION) {
      if (target_width_ % 2 == 1) {
        target_width_ = std::max(2, target_width_ - 1);
      }
      if (target_height_ % 2 == 1) {
        target_height_ = std::max(2, target_height_ - 1);
      }
      target_aspect_ratio_ =
          static_cast<double>(target_width_) / target_height_;
    }
    // Set keyframe width/height for feature upscaling (overwritten by keyframe
    // input if provided).
    if (options_.has_video_features_width() &&
        options_.has_video_features_height()) {
      key_frame_width_ = options_.video_features_width();
      key_frame_height_ = options_.video_features_height();
    } else if (!cc->Inputs().HasTag(kInputKeyFrames)) {
      key_frame_width_ = frame_width_;
      key_frame_height_ = frame_height_;
    }
    // Check provided dimensions.
    RET_CHECK_GT(target_width_, 0) << "Target width is non-positive.";
    RET_CHECK_NE(target_width_ % 2, 1)
        << "Target width cannot be odd, because encoder expects dimension "
           "values to be even.";
    RET_CHECK_GT(target_height_, 0) << "Target height is non-positive.";
    RET_CHECK_NE(target_height_ % 2, 1)
        << "Target height cannot be odd, because encoder expects dimension "
           "values to be even.";
  }

  // Sets key frame dimension.
  if (cc->Inputs().HasTag(kInputKeyFrames) &&
      !cc->Inputs().Tag(kInputKeyFrames).Value().IsEmpty() &&
      key_frame_width_ < 0) {
    const auto& key_frame = cc->Inputs().Tag(kInputKeyFrames).Get<ImageFrame>();
    key_frame_width_ = key_frame.Width();
    key_frame_height_ = key_frame.Height();
  }

  // Processes a scene when shot boundary or buffer is full.
  bool is_end_of_scene = false;
  if (!cc->Inputs().Tag(kInputShotBoundaries).Value().IsEmpty()) {
    is_end_of_scene = cc->Inputs().Tag(kInputShotBoundaries).Get<bool>();
  }
  const bool force_buffer_flush =
      scene_frames_.size() >= options_.max_scene_size();
  if (!scene_frames_.empty() && (is_end_of_scene || force_buffer_flush)) {
    MP_RETURN_IF_ERROR(ProcessScene(is_end_of_scene, cc));
  }

  // Saves frame and timestamp and whether it is a key frame.
  if (!cc->Inputs().Tag(kInputVideoFrames).Value().IsEmpty()) {
    LOG_EVERY_N(ERROR, 10)
        << "------------------------ (Breathing) Time(s): "
        << cc->Inputs().Tag(kInputVideoFrames).Value().Timestamp().Seconds();
    const auto& frame = cc->Inputs().Tag(kInputVideoFrames).Get<ImageFrame>();
    const cv::Mat frame_mat = formats::MatView(&frame);
    cv::Mat copy_mat;
    frame_mat.copyTo(copy_mat);
    scene_frames_.push_back(copy_mat);
    scene_frame_timestamps_.push_back(cc->InputTimestamp().Value());
    is_key_frames_.push_back(
        !cc->Inputs().Tag(kInputDetections).Value().IsEmpty());
  }

  // Packs key frame info.
  if (!cc->Inputs().Tag(kInputDetections).Value().IsEmpty()) {
    const auto& detections =
        cc->Inputs().Tag(kInputDetections).Get<DetectionSet>();
    KeyFrameInfo key_frame_info;
    MP_RETURN_IF_ERROR(PackKeyFrameInfo(
        cc->InputTimestamp().Value(), detections, frame_width_, frame_height_,
        key_frame_width_, key_frame_height_, &key_frame_info));
    key_frame_infos_.push_back(key_frame_info);
  }

  // Buffers static features.
  if (cc->Inputs().HasTag(kInputStaticFeatures) &&
      !cc->Inputs().Tag(kInputStaticFeatures).Value().IsEmpty()) {
    static_features_.push_back(
        cc->Inputs().Tag(kInputStaticFeatures).Get<StaticFeatures>());
    static_features_timestamps_.push_back(cc->InputTimestamp().Value());
  }

  return ::mediapipe::OkStatus();
}

::mediapipe::Status SceneCroppingCalculator::Close(
    ::mediapipe::CalculatorContext* cc) {
  if (!scene_frames_.empty()) {
    MP_RETURN_IF_ERROR(ProcessScene(/* is_end_of_scene = */ true, cc));
  }
  if (cc->Outputs().HasTag(kOutputSummary)) {
    cc->Outputs()
        .Tag(kOutputSummary)
        .Add(summary_.release(), Timestamp::PostStream());
  }
  return ::mediapipe::OkStatus();
}

::mediapipe::Status SceneCroppingCalculator::RemoveStaticBorders() {
  int top_border_size = 0, bottom_border_size = 0;
  MP_RETURN_IF_ERROR(ComputeSceneStaticBordersSize(
      static_features_, &top_border_size, &bottom_border_size));
  const double scale = static_cast<double>(frame_height_) / key_frame_height_;
  top_border_distance_ = std::round(scale * top_border_size);
  const int bottom_border_distance = std::round(scale * bottom_border_size);
  effective_frame_height_ =
      frame_height_ - top_border_distance_ - bottom_border_distance;

  if (top_border_distance_ > 0 || bottom_border_distance > 0) {
    VLOG(1) << "Remove top border " << top_border_distance_ << " bottom border "
            << bottom_border_distance;
    // Remove borders from frames.
    cv::Rect roi(0, top_border_distance_, frame_width_,
                 effective_frame_height_);
    for (int i = 0; i < scene_frames_.size(); ++i) {
      cv::Mat tmp;
      scene_frames_[i](roi).copyTo(tmp);
      scene_frames_[i] = tmp;
    }
    // Adjust detection bounding boxes.
    for (int i = 0; i < key_frame_infos_.size(); ++i) {
      DetectionSet adjusted_detections;
      const auto& detections = key_frame_infos_[i].detections();
      for (int j = 0; j < detections.detections_size(); ++j) {
        const auto& detection = detections.detections(j);
        SalientRegion adjusted_detection = detection;
        // Clamp the box to be within the de-bordered frame.
        if (!ClampRect(0, top_border_distance_, frame_width_,
                       top_border_distance_ + effective_frame_height_,
                       adjusted_detection.mutable_location())
                 .ok()) {
          continue;
        }
        // Offset the y position.
        adjusted_detection.mutable_location()->set_y(
            adjusted_detection.location().y() - top_border_distance_);
        *adjusted_detections.add_detections() = adjusted_detection;
      }
      *key_frame_infos_[i].mutable_detections() = adjusted_detections;
    }
  }
  return ::mediapipe::OkStatus();
}

::mediapipe::Status
SceneCroppingCalculator::InitializeFrameCropRegionComputer() {
  key_frame_crop_options_ = options_.key_frame_crop_options();
  MP_RETURN_IF_ERROR(
      SetKeyFrameCropTarget(frame_width_, effective_frame_height_,
                            target_aspect_ratio_, &key_frame_crop_options_));
  VLOG(1) << "Target width " << key_frame_crop_options_.target_width();
  VLOG(1) << "Target height " << key_frame_crop_options_.target_height();
  frame_crop_region_computer_ =
      absl::make_unique<FrameCropRegionComputer>(key_frame_crop_options_);
  return ::mediapipe::OkStatus();
}

void SceneCroppingCalculator::FilterKeyFrameInfo() {
  if (!options_.user_hint_override()) {
    return;
  }
  std::vector<KeyFrameInfo> user_hints_only;
  bool has_user_hints = false;
  for (auto key_frame : key_frame_infos_) {
    DetectionSet user_hint_only_set;
    for (const auto& detection : key_frame.detections().detections()) {
      if (detection.signal_type().has_standard() &&
          detection.signal_type().standard() == SignalType::USER_HINT) {
        *user_hint_only_set.add_detections() = detection;
        has_user_hints = true;
      }
    }
    *key_frame.mutable_detections() = user_hint_only_set;
    user_hints_only.push_back(key_frame);
  }
  if (has_user_hints) {
    key_frame_infos_ = user_hints_only;
  }
}

::mediapipe::Status SceneCroppingCalculator::ProcessScene(
    const bool is_end_of_scene, CalculatorContext* cc) {
  // Removes detections under special circumstances.
  FilterKeyFrameInfo();

  // Removes any static borders.
  MP_RETURN_IF_ERROR(RemoveStaticBorders());

  // Decides if solid background color padding is possible and sets up color
  // interpolation functions in CIELAB. Uses linear interpolation by default.
  MP_RETURN_IF_ERROR(FindSolidBackgroundColor(
      static_features_, static_features_timestamps_,
      options_.solid_background_frames_padding_fraction(),
      &has_solid_background_, &background_color_l_function_,
      &background_color_a_function_, &background_color_b_function_));

  // Computes key frame crop regions.
  MP_RETURN_IF_ERROR(InitializeFrameCropRegionComputer());
  const int num_key_frames = key_frame_infos_.size();
  std::vector<KeyFrameCropResult> key_frame_crop_results(num_key_frames);
  for (int i = 0; i < num_key_frames; ++i) {
    MP_RETURN_IF_ERROR(frame_crop_region_computer_->ComputeFrameCropRegion(
        key_frame_infos_[i], &key_frame_crop_results[i]));
  }

  // Analyzes scene camera motion and generates FocusPointFrames.
  auto analyzer_options = options_.scene_camera_motion_analyzer_options();
  analyzer_options.set_allow_sweeping(analyzer_options.allow_sweeping() &&
                                      !has_solid_background_);
  scene_camera_motion_analyzer_ =
      absl::make_unique<SceneCameraMotionAnalyzer>(analyzer_options);
  SceneKeyFrameCropSummary scene_summary;
  std::vector<FocusPointFrame> focus_point_frames;
  SceneCameraMotion scene_camera_motion;
  MP_RETURN_IF_ERROR(
      scene_camera_motion_analyzer_->AnalyzeSceneAndPopulateFocusPointFrames(
          key_frame_infos_, key_frame_crop_options_, key_frame_crop_results,
          frame_width_, effective_frame_height_, scene_frame_timestamps_,
          &scene_summary, &focus_point_frames, &scene_camera_motion));

  // Crops scene frames.
  std::vector<cv::Mat> cropped_frames;
  MP_RETURN_IF_ERROR(scene_cropper_->CropFrames(
      scene_summary, scene_frames_, focus_point_frames,
      prior_focus_point_frames_, output_file_path_, &cropped_frames));

  bool apply_padding = false;

  // Caches prior FocusPointFrames if this was not the end of a scene.
  prior_focus_point_frames_.clear();
  if (!is_end_of_scene) {
    const int start = std::max(0, static_cast<int>(scene_frames_.size()) -
                                      options_.prior_frame_buffer_size());
    for (int i = start; i < num_key_frames; ++i) {
      prior_focus_point_frames_.push_back(focus_point_frames[i]);
    }
  }

  // Optionally outputs visualization frames.
  MP_RETURN_IF_ERROR(OutputVizFrames(key_frame_crop_results, focus_point_frames,
                                     scene_summary.crop_window_width(),
                                     scene_summary.crop_window_height(), cc));

  const double start_sec = Timestamp(scene_frame_timestamps_.front()).Seconds();
  const double end_sec = Timestamp(scene_frame_timestamps_.back()).Seconds();
  VLOG(1) << absl::StrFormat("Processed a scene from %.2f sec to %.2f sec",
                             start_sec, end_sec);

  // Optionally makes summary.
  if (cc->Outputs().HasTag(kOutputSummary)) {
    auto* scene_summary = summary_->add_scene_summaries();
    scene_summary->set_start_sec(start_sec);
    scene_summary->set_end_sec(end_sec);
    *(scene_summary->mutable_camera_motion()) = scene_camera_motion;
    scene_summary->set_is_end_of_scene(is_end_of_scene);
    scene_summary->set_is_padded(apply_padding);
  }

  key_frame_infos_.clear();
  scene_frames_.clear();
  scene_frame_timestamps_.clear();
  is_key_frames_.clear();
  static_features_.clear();
  static_features_timestamps_.clear();
  return ::mediapipe::OkStatus();
}

mediapipe::Status SceneCroppingCalculator::OutputVizFrames(
    const std::vector<KeyFrameCropResult>& key_frame_crop_results,
    const std::vector<FocusPointFrame>& focus_point_frames,
    const int crop_window_width, const int crop_window_height,
    CalculatorContext* cc) const {
  if (cc->Outputs().HasTag(kOutputKeyFrameCropViz)) {
    std::vector<std::unique_ptr<ImageFrame>> viz_frames;
    MP_RETURN_IF_ERROR(DrawDetectionsAndCropRegions(
        scene_frames_, is_key_frames_, key_frame_infos_, key_frame_crop_results,
        frame_format_, &viz_frames));
    for (int i = 0; i < scene_frames_.size(); ++i) {
      cc->Outputs()
          .Tag(kOutputKeyFrameCropViz)
          .Add(viz_frames[i].release(), Timestamp(scene_frame_timestamps_[i]));
    }
  }
  if (cc->Outputs().HasTag(kOutputFocusPointFrameViz)) {
    std::vector<std::unique_ptr<ImageFrame>> viz_frames;
    MP_RETURN_IF_ERROR(DrawFocusPointAndCropWindow(
        scene_frames_, focus_point_frames, options_.viz_overlay_opacity(),
        crop_window_width, crop_window_height, frame_format_, &viz_frames));
    for (int i = 0; i < scene_frames_.size(); ++i) {
      cc->Outputs()
          .Tag(kOutputFocusPointFrameViz)
          .Add(viz_frames[i].release(), Timestamp(scene_frame_timestamps_[i]));
    }
  }
  return ::mediapipe::OkStatus();
}

REGISTER_CALCULATOR(SceneCroppingCalculator);

}  // namespace autoflip
}  // namespace mediapipe
