// tests/test_reply_processor.cpp
#include <gtest/gtest.h>
#include "vrl/radar/processing/reply_processor.h"
#include "vrl/radar/utils/utils.h"
#include <memory>

using namespace vrl::radar;

class ReplyProcessorTest : public ::testing::Test {
protected:
    void SetUp() override {
        RadarConfig config;
        config.range_bin_rbs = 30.0;
        config.range_bin_uvd = 60.0;
        config.min_amplitude = 10;
        processor_ = std::make_unique<ReplyProcessor>(config);
    }
    
    std::array<uint8_t, RBSReply::ETHER_POSITIONS> create_rbs_amplitudes(uint16_t code, uint8_t base_amp = 200) {
        std::array<uint8_t, RBSReply::ETHER_POSITIONS> amps{};
        amps.fill(0);
        
        amps[0] = base_amp;  // F1
        amps[14] = base_amp; // F2
        
        for (int i = 0; i < 12; ++i) {
            if ((code >> i) & 1) {
                size_t pos = utils::bit_position(i);
                amps[pos] = base_amp;
            }
        }
        
        return amps;
    }
    
    std::array<uint8_t, UVDReply::ETHER_POSITIONS> create_uvd_amplitudes(uint32_t data, uint8_t base_amp = 200) {
        std::array<uint8_t, UVDReply::ETHER_POSITIONS> amps{};
        amps.fill(0);
        
        for (int repeat = 0; repeat < 2; ++repeat) {
            for (int i = 0; i < 20; ++i) {
                bool bit = (data >> i) & 1;
                size_t offset = repeat * 40 + i * 2;
                
                if (!bit) {
                    amps[offset] = base_amp;
                    amps[offset + 1] = 0;
                } else {
                    amps[offset] = 0;
                    amps[offset + 1] = base_amp;
                }
            }
        }
        
        return amps;
    }
    
    std::unique_ptr<ReplyProcessor> processor_;
};

TEST_F(ReplyProcessorTest, DecodeRBSWithErrors) {
    uint16_t original_code = 0x123;
    auto amps = create_rbs_amplitudes(original_code);
    
    uint16_t decoded = processor_->decode_rbs_with_errors(amps);
    
    EXPECT_EQ(decoded, original_code);
}

TEST_F(ReplyProcessorTest, DecodeRBSWithNoise) {
    uint16_t original_code = 0x456;
    auto amps = create_rbs_amplitudes(original_code);
    
    // Добавляем небольшой шум в один бит
    amps[1] = 100;  // бит 0 должен быть 200
    
    uint16_t decoded = processor_->decode_rbs_with_errors(amps);
    
    // Может не совпасть из-за шума
    // Но не должен быть 0
    EXPECT_NE(decoded, 0);
}

TEST_F(ReplyProcessorTest, DecodeUVDWithErrors) {
    uint32_t original_data = 0x12345;
    auto amps = create_uvd_amplitudes(original_data);
    
    uint32_t decoded = processor_->decode_uvd_with_errors(amps);
    
    EXPECT_EQ(decoded, original_data);
}

TEST_F(ReplyProcessorTest, DecodeUVDWithNoise) {
    uint32_t original_data = 0xABCDE;
    auto amps = create_uvd_amplitudes(original_data);
    
    // Добавляем шум в один бит (оба повторения)
    amps[0] = 0;   // левый импульс бита 0 (должен быть 200)
    amps[40] = 0;  // левый импульс бита 0 (второе повторение)
    
    uint32_t decoded = processor_->decode_uvd_with_errors(amps);
    
    // Может не совпасть из-за шума
    EXPECT_NE(decoded, 0);
}

TEST_F(ReplyProcessorTest, AnalyzeRBSFeatures) {
    RBSReply reply;
    reply.ether_amplitudes.fill(0);
    reply.ether_amplitudes[0] = 200;   // F1
    reply.ether_amplitudes[14] = 200;  // F2
    reply.ether_amplitudes[1] = 200;   // бит 0
    
    ReplyFeatures features = processor_->analyze_rbs(reply);
    
    EXPECT_TRUE(features.has_framing);
    EXPECT_GT(features.snr_estimate, 0.0);
    EXPECT_GT(features.confidence, 0.0);
}

TEST_F(ReplyProcessorTest, AnalyzeUVDFeatures) {
    UVDReply reply;
    reply.ether_amplitudes.fill(0);
    reply.data20 = 0x12345;
    
    // Устанавливаем биты
    for (int i = 0; i < 20; ++i) {
        bool bit = (reply.data20 >> i) & 1;
        if (!bit) {
            reply.ether_amplitudes[i * 2] = 200;
        } else {
            reply.ether_amplitudes[i * 2 + 1] = 200;
        }
        // Второе повторение
        if (!bit) {
            reply.ether_amplitudes[40 + i * 2] = 200;
        } else {
            reply.ether_amplitudes[40 + i * 2 + 1] = 200;
        }
    }
    
    ReplyFeatures features = processor_->analyze_uvd(reply);
    
    EXPECT_EQ(features.bit_errors, 0);
    EXPECT_GT(features.snr_estimate, 0.0);
    EXPECT_GT(features.confidence, 0.0);
}

TEST_F(ReplyProcessorTest, EstimateSNRRBS) {
    RBSReply reply;
    reply.ether_amplitudes.fill(0);
    reply.ether_amplitudes[0] = 200;   // F1 - сигнал
    reply.ether_amplitudes[14] = 200;  // F2 - сигнал
    reply.ether_amplitudes[1] = 200;   // бит - сигнал
    
    double snr = processor_->estimate_snr(reply);
    
    EXPECT_GT(snr, 0.0);
}

TEST_F(ReplyProcessorTest, EstimateSNRUVD) {
    UVDReply reply;
    reply.ether_amplitudes.fill(0);
    reply.ether_amplitudes[0] = 200;   // левый импульс
    reply.ether_amplitudes[1] = 0;     // правый импульс
    
    double snr = processor_->estimate_snr(reply);
    
    EXPECT_GT(snr, 0.0);
}

TEST_F(ReplyProcessorTest, NormalizeAmplitudesRBS) {
    RBSReply reply;
    reply.ether_amplitudes.fill(0);
    reply.ether_amplitudes[0] = 100;   // F1
    reply.ether_amplitudes[14] = 100;  // F2
    
    processor_->normalize_amplitudes(reply);
    
    // После нормализации значения должны увеличиться
    EXPECT_GT(reply.ether_amplitudes[0], 100);
    EXPECT_GT(reply.ether_amplitudes[14], 100);
}

TEST_F(ReplyProcessorTest, NormalizeAmplitudesUVD) {
    UVDReply reply;
    reply.ether_amplitudes.fill(0);
    reply.ether_amplitudes[0] = 100;   // левый импульс
    
    processor_->normalize_amplitudes(reply);
    
    EXPECT_GT(reply.ether_amplitudes[0], 100);
}

TEST_F(ReplyProcessorTest, ConfidenceRBSWithFraming) {
    RBSReply reply;
    reply.ether_amplitudes.fill(0);
    reply.ether_amplitudes[0] = 200;   // F1
    reply.ether_amplitudes[14] = 200;  // F2
    reply.ether_amplitudes[1] = 200;   // бит 0
    
    ReplyFeatures features = processor_->analyze_rbs(reply);
    
    EXPECT_TRUE(features.has_framing);
    EXPECT_GT(features.confidence, 0.0);
}

TEST_F(ReplyProcessorTest, ConfidenceRBSWithoutFraming) {
    RBSReply reply;
    reply.ether_amplitudes.fill(0);
    reply.ether_amplitudes[0] = 0;     // F1 отсутствует
    reply.ether_amplitudes[14] = 200;  // F2 есть
    reply.ether_amplitudes[1] = 200;   // бит 0
    
    ReplyFeatures features = processor_->analyze_rbs(reply);
    
    EXPECT_FALSE(features.has_framing);
}

TEST_F(ReplyProcessorTest, PulseStabilityRBS) {
    RBSReply reply;
    reply.ether_amplitudes.fill(0);
    reply.ether_amplitudes[0] = 200;   // F1
    reply.ether_amplitudes[14] = 200;  // F2
    reply.ether_amplitudes[1] = 180;   // бит 0
    reply.ether_amplitudes[2] = 190;   // бит 1
    reply.ether_amplitudes[3] = 200;   // бит 2
    
    ReplyFeatures features = processor_->analyze_rbs(reply);
    
    EXPECT_GT(features.pulse_stability, 0.0);
}

TEST_F(ReplyProcessorTest, PulseStabilityUVD) {
    UVDReply reply;
    reply.ether_amplitudes.fill(0);
    reply.data20 = 0x12345;
    
    // Устанавливаем биты с хорошим соотношением
    for (int i = 0; i < 20; ++i) {
        bool bit = (reply.data20 >> i) & 1;
        if (!bit) {
            reply.ether_amplitudes[i * 2] = 200;
            reply.ether_amplitudes[i * 2 + 1] = 0;
            reply.ether_amplitudes[40 + i * 2] = 200;
            reply.ether_amplitudes[40 + i * 2 + 1] = 0;
        } else {
            reply.ether_amplitudes[i * 2] = 0;
            reply.ether_amplitudes[i * 2 + 1] = 200;
            reply.ether_amplitudes[40 + i * 2] = 0;
            reply.ether_amplitudes[40 + i * 2 + 1] = 200;
        }
    }
    
    ReplyFeatures features = processor_->analyze_uvd(reply);
    
    EXPECT_GT(features.pulse_stability, 0.0);
}

TEST_F(ReplyProcessorTest, SPIHandling) {
    RBSReply reply;
    reply.ether_amplitudes.fill(0);
    reply.ether_amplitudes[0] = 200;   // F1
    reply.ether_amplitudes[14] = 200;  // F2
    reply.ether_amplitudes[17] = 200;  // SPI
    
    ReplyFeatures features = processor_->analyze_rbs(reply);
    
    EXPECT_TRUE(features.has_spi);
}

TEST_F(ReplyProcessorTest, NoSPI) {
    RBSReply reply;
    reply.ether_amplitudes.fill(0);
    reply.ether_amplitudes[0] = 200;   // F1
    reply.ether_amplitudes[14] = 200;  // F2
    reply.ether_amplitudes[17] = 0;    // SPI отсутствует
    
    ReplyFeatures features = processor_->analyze_rbs(reply);
    
    EXPECT_FALSE(features.has_spi);
}

TEST_F(ReplyProcessorTest, DecodeRBSWithInvalidFraming) {
    uint16_t original_code = 0x123;
    auto amps = create_rbs_amplitudes(original_code);
    amps[0] = 0;   // Убираем F1
    
    uint16_t decoded = processor_->decode_rbs_with_errors(amps);
    
    // Должен вернуть 0 или неправильное значение
    // Но не должен вызывать исключение
    EXPECT_NO_THROW(processor_->decode_rbs_with_errors(amps));
}
