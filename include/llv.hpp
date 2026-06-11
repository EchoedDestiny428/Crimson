#ifndef LLV_HPP_
#define LLV_HPP_

#include "main.h"

namespace llv {

class LimelightVision {
	public:
	explicit LimelightVision(std::uint8_t port = 1, std::uint8_t signature_id = 1,
	                         vision_zero_e_t zero_point = E_VISION_ZERO_TOPLEFT);

	void configure_defaults(std::uint8_t exposure = 50, bool auto_white_balance = false) const;
	void set_signature_id(std::uint8_t signature_id);
	std::uint8_t get_signature_id() const;

	void set_exposure(std::uint8_t exposure) const;
	void set_auto_white_balance(bool enabled) const;
	void clear_led() const;
	void set_led(std::int32_t rgb) const;

	void update();

	bool has_target() const;
	int get_tv() const;
	double get_tx() const;
	double get_ty() const;
	double get_ta() const;
	double get_ts() const;
	std::uint32_t get_data_age_ms() const;

	vision_object_s_t get_raw_target() const;
	pros::Vision &sensor();

	private:
	static constexpr double kFrameWidth = 316.0;
	static constexpr double kFrameHeight = 212.0;
	static constexpr double kCenterX = kFrameWidth / 2.0;
	static constexpr double kCenterY = kFrameHeight / 2.0;

	static bool is_valid_object(const vision_object_s_t &object);

	pros::Vision sensor_;
	std::uint8_t signature_id_;
	vision_object_s_t target_;
	bool target_valid_;
	std::uint32_t last_update_ms_;
};

}  // namespace llv

#endif