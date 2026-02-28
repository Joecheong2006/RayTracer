#pragma once
#include "gpu/Buffer.h"
#include <vector>
#include <string>
#include <cstring>

namespace gpu {
    class FloatBuffer : public Buffer {
        std::vector<f32> m_data;
        std::string m_name;

        template <typename T>
        static f32 bitsToFloat(T &val) {
            f32 f;
            std::memcpy(&f, &val, sizeof(T));
            return f;
        }

    public:
        virtual void push(const glm::vec3 &v) override { m_data.insert(m_data.end(), &v.x, &v.x + 3); }
        virtual void push(const glm::ivec3 &iv) override {
            m_data.push_back(*reinterpret_cast<const f32*>(&iv.x));
            m_data.push_back(*reinterpret_cast<const f32*>(&iv.y));
            m_data.push_back(*reinterpret_cast<const f32*>(&iv.z));
        }
        virtual void push(const glm::vec2 &v) override { m_data.insert(m_data.end(), &v.x, &v.x + 2); }
        virtual void push(f32 val) override { m_data.push_back(val); }
        virtual void push(i32 val) override { m_data.push_back(bitsToFloat(val)); }
        virtual void push(u32 val) override { m_data.push_back(bitsToFloat(val)); }
        virtual void push(bool val) override { m_data.push_back(val); };
        virtual void push(const char *data, size_t size) override {
            const f32 *fdata = reinterpret_cast<const f32*>(data);
            m_data.insert(m_data.end(), fdata, fdata + size / sizeof(f32));
        }

        virtual const void* data() const override { return m_data.data(); }
        virtual size_t size() const override { return m_data.size() * sizeof(f32); }
        virtual void clear() override { m_data.clear(); }

    };
}

