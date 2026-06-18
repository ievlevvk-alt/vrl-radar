// include/vrl/radar/processing/garbling_solver.h
#pragma once

#include "../core/types.h"
#include "../core/replies.h"
#include <vector>
#include <array>
#include <optional>
#include <memory>
#include <string>

namespace vrl {
namespace radar {

// ============================================================================
// RESULT STRUCTURE
// ============================================================================

template<typename ReplyType>
struct SeparationResult {
    std::vector<ReplyType> separated_replies;
    double confidence{0.0};
    bool ambiguous{false};
    std::string method_used;
};

// ============================================================================
// BASE GARBLING SOLVER
// ============================================================================

class GarblingSolver {
public:
    GarblingSolver(const RadarConfig& config);
    virtual ~GarblingSolver() = default;
    
    virtual SeparationResult<RBSReply> separate_rbs(
        const std::vector<RBSReply>& mixture,
        const std::vector<uint16_t>& expected_codes = {}) = 0;
    
    virtual SeparationResult<UVDReply> separate_uvd(
        const std::vector<UVDReply>& mixture,
        const std::vector<uint32_t>& expected_data = {}) = 0;
    
    void set_debug(bool enable) { debug_ = enable; }
    
protected:
    RadarConfig config_;
    bool debug_{false};
    void log(const std::string& msg) const;
};

// ============================================================================
// THRESHOLD GARBLING SOLVER
// ============================================================================

class ThresholdGarblingSolver : public GarblingSolver {
public:
    ThresholdGarblingSolver(const RadarConfig& config, uint8_t threshold = 50);
    
    SeparationResult<RBSReply> separate_rbs(
        const std::vector<RBSReply>& mixture,
        const std::vector<uint16_t>& expected_codes = {}) override;
    
    SeparationResult<UVDReply> separate_uvd(
        const std::vector<UVDReply>& mixture,
        const std::vector<uint32_t>& expected_data = {}) override;
    
private:
    uint8_t threshold_;
    
    std::vector<uint16_t> detect_possible_codes_rbs(
        const std::array<uint8_t, RBSReply::ETHER_POSITIONS>& mixture);
    
    bool check_code_presence_rbs(
        uint16_t code,
        const std::array<uint8_t, RBSReply::ETHER_POSITIONS>& mixture);
    
    std::vector<uint32_t> detect_possible_data_uvd(
        const std::array<uint8_t, UVDReply::ETHER_POSITIONS>& mixture);
};

// ============================================================================
// ITERATIVE SUBTRACTION GARBLING SOLVER
// ============================================================================

class IterativeSubtractionSolver : public GarblingSolver {
public:
    IterativeSubtractionSolver(const RadarConfig& config, 
                               double min_amplitude_ratio = 0.3,
                               int max_iterations = 5);
    
    SeparationResult<RBSReply> separate_rbs(
        const std::vector<RBSReply>& mixture,
        const std::vector<uint16_t>& expected_codes = {}) override;
    
    SeparationResult<UVDReply> separate_uvd(
        const std::vector<UVDReply>& mixture,
        const std::vector<uint32_t>& expected_data = {}) override;
    
private:
    double min_amplitude_ratio_;
    int max_iterations_;
    
    template<typename ReplyType>
    std::optional<ReplyType> find_dominant_reply(
        const std::vector<ReplyType>& mixture);
    
    template<typename ReplyType>
    std::vector<ReplyType> subtract_reply(
        const std::vector<ReplyType>& mixture,
        const ReplyType& to_subtract);
    
    double evaluate_separation_quality(
        const std::vector<RBSReply>& original,
        const std::vector<RBSReply>& separated);
};

} // namespace radar
} // namespace vrl
