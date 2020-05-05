/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2017-2019 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

//#ifndef IVW_MORPHVIS_BATCHVOLUMESAMPLER_H
//#define IVW_MORPHVIS_BATCHVOLUMESAMPLER_H
//
//#include <modules/morphvis/morphvismoduledefine.h>
#include <inviwo/core/common/inviwo.h>

#include <inviwo/core/util/indexmapper.h>
#include <inviwo/core/util/interpolation.h>
#include <inviwo/core/datastructures/volume/volumeramprecision.h>

#include <array>

namespace inviwo {

enum class BoundaryBehaviour { Clamp, Repeat };

enum class SampleBehaviour {
    Nearest,
    Trilinear,
};

namespace detail {

template <size_t DataDims, typename T, SampleBehaviour SB, BoundaryBehaviour BX,
          BoundaryBehaviour BY, BoundaryBehaviour BZ>
struct Sample {
    static void sample(const std::vector<dvec3>& pos,
                       std::vector<Vector<DataDims, double>>& samples, const T* data, size3_t dims);
};

template <size_t DataDims, typename T, BoundaryBehaviour BX, BoundaryBehaviour BY,
          BoundaryBehaviour BZ>
struct Sample<DataDims, T, SampleBehaviour::Trilinear, BX, BY, BZ> {
    static void sample(const std::vector<dvec3>& pos, std::vector<Vector<DataDims, double>>& result,
                       const T* data, glm::i64vec3 dims) {
        auto posit = pos.begin();
        auto resultit = result.begin();
        auto posend = pos.end();

        const dvec3 ddims{dims};
        const ivec3 dimsi{dims};
        const dvec3 ddimsInv{1.0 / ddims};
        const bvec3 boundarymix{BX == BoundaryBehaviour::Clamp, BY == BoundaryBehaviour::Clamp,
                                BZ == BoundaryBehaviour::Clamp};

        const std::array<ivec3, 8> offsets = {{{0, 0, 0},
                                               {1, 0, 0},
                                               {0, 1, 0},
                                               {1, 1, 0},
                                               {0, 0, 1},
                                               {1, 0, 1},
                                               {0, 1, 1},
                                               {1, 1, 1}}};
        std::array<Vector<DataDims, double>, 8> samples;

        auto modInt = [](int a, int b) {
            int result = a % b;
            //int result = a - b * (a / b);
            // take care of negative values
            //return result + (result < 0 ? b : 0);
            return result + ((result >> 31) & b);
        };

        const util::IndexMapper<3, int> im(dims);
        for (; posit != posend; ++posit, ++resultit) {
            const dvec3 voxelPos{*posit * ddims};
            const dvec3 voxelFloor{glm::floor(voxelPos)};
            const ivec3 voxelPosi{voxelFloor};
            const dvec3 interpolants{voxelPos - voxelFloor};

            size_t i{0};
            for (const auto& offset : offsets) {
                const auto pOffset(voxelPosi + offset);
                ivec3 indexPos;
                for (size_t j = 0; j < 3; ++j) {
                    if (boundarymix[i]) {
                        indexPos[j] = glm::clamp(pOffset[j], 0, dimsi[j] - 1);
                    } else {
                        indexPos[j] = modInt(pOffset[j], dimsi[j]);
                    }
                }
                samples[i++] = util::glm_convert<Vector<DataDims, double>>(data[im(indexPos)]);
            }

            *resultit =
                Interpolation<Vector<DataDims, double>>::trilinear(samples.data(), interpolants);
        }
    }
};

template <size_t DataDims, typename T, BoundaryBehaviour BX, BoundaryBehaviour BY,
          BoundaryBehaviour BZ>
struct Sample<DataDims, T, SampleBehaviour::Nearest, BX, BY, BZ> {
    static void sample(const std::vector<dvec3>& pos, std::vector<Vector<DataDims, double>>& result,
                       const T* data, size3_t dims) {
        auto posit = pos.begin();
        auto resultit = result.begin();
        auto posend = pos.end();

        const util::IndexMapper<3, int> im(ivec3{dims});

        const dvec3 ddims{dims};
        const dvec3 ddimsInv{1.0 / ddims};
        const dvec3 mix{BX == BoundaryBehaviour::Clamp, BY == BoundaryBehaviour::Clamp,
                        BZ == BoundaryBehaviour::Clamp};

        for (; posit != posend; ++posit, ++resultit) {
            const auto p{glm::mix(*posit - glm::floor(*posit),
                                  glm::clamp(*posit, dvec3{0.0}, dvec3{1.0} - ddimsInv), mix)};
            const auto indexPos = ivec3{glm::floor(p * ddims)};
            const int ind = im(indexPos);
            *resultit = util::glm_convert<Vector<DataDims, double>>(data[ind]);
        }
    }
};

}  // namespace detail

template <size_t DataDims>
class BatchVolumeSampler {
public:
    virtual void sample(const std::vector<dvec3>& pos,
                        std::vector<Vector<DataDims, double>>& samples) = 0;
};

template <size_t DataDims, typename T, SampleBehaviour SB, BoundaryBehaviour BX,
          BoundaryBehaviour BY, BoundaryBehaviour BZ>
class TemplateBatchVolumeSampler : public BatchVolumeSampler<DataDims> {
public:
    TemplateBatchVolumeSampler(std::shared_ptr<const Volume> volume,
                               const VolumeRAMPrecision<T>* vrprecision)
        : BatchVolumeSampler(), volume_(volume), vrprecision_(vrprecision) {}

    virtual ~TemplateBatchVolumeSampler() = default;

    virtual void sample(const std::vector<dvec3>& pos,
                        std::vector<Vector<DataDims, double>>& samples) override {
        const glm::i64vec3 dims{volume_->getDimensions()};
        const auto data = vrprecision_->getDataTyped();
        detail::Sample<DataDims, T, SB, BX, BY, BZ>::sample(pos, samples, data, dims);
    }

private:
    std::shared_ptr<const Volume> volume_;
    const VolumeRAMPrecision<T>* vrprecision_;
};

namespace util {

template <size_t DataDims, SampleBehaviour SB = SampleBehaviour::Trilinear,
          BoundaryBehaviour BX = BoundaryBehaviour::Clamp,
          BoundaryBehaviour BY = BoundaryBehaviour::Clamp,
          BoundaryBehaviour BZ = BoundaryBehaviour::Clamp>
std::unique_ptr<BatchVolumeSampler<DataDims>> makeBatchVolumeSampler(
    std::shared_ptr<const Volume> volume) {
    return volume->getRepresentation<VolumeRAM>()
        ->dispatch<std::unique_ptr<BatchVolumeSampler<DataDims>>, dispatching::filter::All>(
            [volume](const auto vrprecision) {
                using T = util::PrecisionValueType<decltype(vrprecision)>;
                return std::unique_ptr<BatchVolumeSampler<DataDims>>(
                    new TemplateBatchVolumeSampler<DataDims, T, SB, BX, BY, BZ>(volume,
                                                                                vrprecision));
            });
}

}  // namespace util

}  // namespace inviwo

//#endif  // IVW_MORPHVIS_BATCHVOLUMESAMPLER_H
