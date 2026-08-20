#include "crimson.hpp"

namespace crimson {

Crimson::Crimson(std::uint8_t port, std::uint8_t signature_id, vision_zero_e_t zero_point)
    : sensor_(port, zero_point), signature_id_(signature_id), target_{}, target_valid_(false), last_update_ms_(0) {
	target_.signature = VISION_OBJECT_ERR_SIG;
}

void Crimson::configure_defaults(std::uint8_t exposure, bool auto_white_balance) const {
	set_auto_white_balance(auto_white_balance);
	set_exposure(exposure);
	clear_led();
}

void Crimson::set_signature_id(std::uint8_t signature_id) {
	signature_id_ = signature_id;
}

std::uint8_t Crimson::get_signature_id() const {
	return signature_id_;
}

void Crimson::set_exposure(std::uint8_t exposure) const {
	sensor_.set_exposure(exposure);
}

void Crimson::set_auto_white_balance(bool enabled) const {
	sensor_.set_auto_white_balance(enabled ? 1U : 0U);
}

void Crimson::clear_led() const {
	sensor_.clear_led();
}

void Crimson::set_led(std::int32_t rgb) const {
	sensor_.set_led(rgb);
}

void Crimson::update() {
	target_ = sensor_.get_by_sig(0, signature_id_);
	target_valid_ = is_valid_object(target_);
	last_update_ms_ = pros::millis();
}

bool Crimson::has_target() const {
	return target_valid_;
}

int Crimson::get_tv() const {
	return target_valid_ ? 1 : 0;
}

double Crimson::get_tx() const {
	if (!target_valid_) {
		return 0.0;
	}

	return static_cast<double>(target_.x_middle_coord) - kCenterX;
}

double Crimson::get_ty() const {
	if (!target_valid_) {
		return 0.0;
	}

	return kCenterY - static_cast<double>(target_.y_middle_coord);
}

double Crimson::get_ta() const {
	if (!target_valid_) {
		return 0.0;
	}

	const double area = static_cast<double>(target_.width) * static_cast<double>(target_.height);
	return (area * 100.0) / (kFrameWidth * kFrameHeight);
}

double Crimson::get_ts() const {
	if (!target_valid_) {
		return 0.0;
	}

	return static_cast<double>(target_.angle) / 10.0;
}

std::uint32_t Crimson::get_data_age_ms() const {
	if (last_update_ms_ == 0) {
		return 0;
	}

	return pros::millis() - last_update_ms_;
}

vision_object_s_t Crimson::get_raw_target() const {
	return target_;
}

pros::Vision &Crimson::sensor() {
	return sensor_;
}

bool Crimson::is_valid_object(const vision_object_s_t &object) {
	return object.signature != VISION_OBJECT_ERR_SIG;
}

}  // namespace crimson
