void OutlierDetection()
{
	const sint32 width = width_;
	const sint32 height = height_;

	const float32& threshold = lrcheck_thres_;

	
	auto& occlusions = occlusions_;
	auto& mismatches = mismatches_;
	occlusions.clear();
	mismatches.clear();

	
	for (sint32 y = 0; y < height; y++) {
		for (sint32 x = 0; x < width; x++) {
			
			auto& disp = disp_left_[y * width + x];
			if (disp == Invalid_Float) {
				mismatches.emplace_back(x, y);
				continue;
			}

			
			const auto col_right = lround(x - disp);
			if (col_right >= 0 && col_right < width) {
				
				const auto& disp_r = disp_right_[y * width + col_right];
				
				if (abs(disp - disp_r) > threshold) {
					
					
					
					
					
					
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

					
					disp = Invalid_Float;
				}
			}
			else {
				
				disp = Invalid_Float;
				mismatches.emplace_back(x, y);
			}
		}
	}
	Refine()
}

void Refine()
{
	if (width_ <= 0 || height_ <= 0 ||
		disp_left_ == nullptr || disp_right_ == nullptr ||
		cost_ == nullptr || cross_arms_ == nullptr) {
		return;
	}

	
	if (do_lr_check_) {
	}
	
	if (do_region_voting_) {
		IterativeRegionVoting();
	}
	
	if (do_interpolating_) {
		ProperInterpolation();
	}
	
	if (do_discontinuity_adjustment_) {
		DepthDiscontinuityAdjustment();
	}

	
	adcensus_util::MedianFilter(disp_left_, disp_left_, width_, height_, 3);
}

