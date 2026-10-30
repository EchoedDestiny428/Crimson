#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace telemetry::wpilog {

inline constexpr std::uint16_t kVersion = 0x0100;
inline constexpr std::uint32_t kControlId = 0;
inline constexpr std::uint8_t kControlStart = 0;
inline constexpr std::size_t kMaxRecordHeaderSize = 1 + 4 + 4 + 8;

constexpr std::size_t byte_width(std::uint64_t value, std::size_t max_bytes) {
    std::size_t bytes = 1;
    while (bytes < max_bytes && (value >> (8 * bytes)) != 0) {
        ++bytes;
    }
    return bytes;
}

constexpr std::size_t put_le(std::uint8_t* out, std::uint64_t value, std::size_t bytes) {
    for (std::size_t i = 0; i < bytes; ++i) {
        out[i] = static_cast<std::uint8_t>(value >> (8 * i));
    }
    return bytes;
}

constexpr std::size_t put_text(std::uint8_t* out, std::string_view text) {
    for (std::size_t i = 0; i < text.size(); ++i) {
        out[i] = static_cast<std::uint8_t>(text[i]);
    }
    return text.size();
}

constexpr std::size_t file_header_size(std::string_view extra_header) {
    return 6 + 2 + 4 + extra_header.size();
}

constexpr std::size_t encode_file_header(std::uint8_t* out, std::string_view extra_header) {
    std::size_t offset = put_text(out, "WPILOG");
    offset += put_le(out + offset, kVersion, 2);
    offset += put_le(out + offset, extra_header.size(), 4);
    offset += put_text(out + offset, extra_header);
    return offset;
}

constexpr std::size_t record_header_size(std::uint32_t id, std::uint32_t size, std::uint64_t timestamp) {
    return 1 + byte_width(id, 4) + byte_width(size, 4) + byte_width(timestamp, 8);
}

constexpr std::size_t encode_record_header(std::uint8_t* out, std::uint32_t id, std::uint32_t size,
                                           std::uint64_t timestamp) {
    const std::size_t id_bytes = byte_width(id, 4);
    const std::size_t size_bytes = byte_width(size, 4);
    const std::size_t timestamp_bytes = byte_width(timestamp, 8);
    out[0] = static_cast<std::uint8_t>((id_bytes - 1) | ((size_bytes - 1) << 2) | ((timestamp_bytes - 1) << 4));
    std::size_t offset = 1;
    offset += put_le(out + offset, id, id_bytes);
    offset += put_le(out + offset, size, size_bytes);
    offset += put_le(out + offset, timestamp, timestamp_bytes);
    return offset;
}

constexpr std::size_t start_payload_size(std::string_view name, std::string_view type) {
    return 1 + 4 + 4 + name.size() + 4 + type.size() + 4;
}

constexpr std::size_t encode_start_payload(std::uint8_t* out, std::uint32_t entry, std::string_view name,
                                           std::string_view type) {
    std::size_t offset = 0;
    out[offset++] = kControlStart;
    offset += put_le(out + offset, entry, 4);
    offset += put_le(out + offset, name.size(), 4);
    offset += put_text(out + offset, name);
    offset += put_le(out + offset, type.size(), 4);
    offset += put_text(out + offset, type);
    offset += put_le(out + offset, 0, 4);
    return offset;
}

}
