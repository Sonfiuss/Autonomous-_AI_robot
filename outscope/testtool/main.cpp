void MultiStepRefiner::OutlierDetection()
{
	const sint32 width = width_;
	const sint32 height = height_;

	const float32& threshold = lrcheck_thres_;

	// 遮挡区像素和误匹配区像素
	auto& occlusions = occlusions_;
	auto& mismatches = mismatches_;
	occlusions.clear();
	mismatches.clear();

	// ---左右一致性检查
	for (sint32 y = 0; y < height; y++) {
		for (sint32 x = 0; x < width; x++) {
			// 左影像视差值
			auto& disp = disp_left_[y * width + x];
			if (disp == Invalid_Float) {
				mismatches.emplace_back(x, y);
				continue;
			}

			// 根据视差值找到右影像上对应的同名像素
			const auto col_right = lround(x - disp);
			if (col_right >= 0 && col_right < width) {
				// 右影像上同名像素的视差值
				const auto& disp_r = disp_right_[y * width + col_right];
				// 判断两个视差值是否一致（差值在阈值内）
				if (abs(disp - disp_r) > threshold) {
					// 区分遮挡区和误匹配区
					// 通过右影像视差算出在左影像的匹配像素，并获取视差disp_rl
					// if(disp_rl > disp) 
					//		pixel in occlusions
					// else 
					//		pixel in mismatches
					const sint32 col_rl = lround(col_right + disp_r);
					if (col_rl > 0 && col_rl < width) {
						const auto& disp_l = disp_left_[y * width + col_rl];
						if (disp_l > disp) {
							occlusions.emplace_back(x, y);
						}
						else {
							mismatches.emplace_back(x, y);
						}
					}
					else {
						mismatches.emplace_back(x, y);
					}

					// 让视差值无效
					disp = Invalid_Float;
				}
			}
			else {
				// 通过视差值在右影像上找不到同名像素（超出影像范围）
				disp = Invalid_Float;
				mismatches.emplace_back(x, y);
			}
		}
	}
}

void MultiStepRefiner::Refine()
{
	if (width_ <= 0 || height_ <= 0 ||
		disp_left_ == nullptr || disp_right_ == nullptr ||
		cost_ == nullptr || cross_arms_ == nullptr) {
		return;
	}

	// step1: outlier detection
	if (do_lr_check_) {
		OutlierDetection();
	}
	// step2: iterative region voting
	if (do_region_voting_) {
		IterativeRegionVoting();
	}
	// step3: proper interpolation
	if (do_interpolating_) {
		ProperInterpolation();
	}
	// step4: discontinuities adjustment
	if (do_discontinuity_adjustment_) {
		DepthDiscontinuityAdjustment();
	}

	// median filter
	adcensus_util::MedianFilter(disp_left_, disp_left_, width_, height_, 3);
}

