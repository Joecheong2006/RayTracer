#pragma once

#include <unordered_map>
#include <vector>
#include "../util.h"
#include "glm/glm.hpp"
#include <string>

namespace gl {
    std::string loadShaderCode(const std::string &path);
    class ShaderProgram {
    private:
        std::unordered_map<std::string, i32> m_uniform_location_cache;
        std::vector<std::string> error;
        u32 m_id;

        u32 compile_shader(const std::string &source, u32 type);
        i32 uniform_location(const std::string &name);

    public:
        ShaderProgram();
        ShaderProgram(const ShaderProgram &shader);
        virtual ~ShaderProgram();

        void create();
        void attachShader(u32 type, const std::string &path);
        void attachShaderCode(u32 type, const std::string &code);
        void link() const;
        void bind() const;
        void unbind() const;
        void release();

        void setUniform4f(const std::string &name, f32 v0, f32 v1, f32 v2, f32 v3);
        void setUniform3f(const std::string &name, const glm::vec3 &v);
        void setUniform3f(const std::string &name, f32 v0, f32 v1, f32 v2);
        void setUniform3f(const std::string &name, f32 *v);
        void setUniform2f(const std::string &name, const glm::vec2 &v);
        void setUniform2f(const std::string &name, f32 *v);
        void setUniform2f(const std::string &name, f32 v0, f32 v1);
        void setUniform1u(const std::string &name, u32 v0);
        void setUniform1i(const std::string &name, i32 v0);
        void setUniform1f(const std::string &name, f32 v0);
        void setUniformMat4(const std::string &name, const glm::mat4 &m);

        std::vector<std::string> getShaderError() const { return error; }
        void clearShaderError() { error.resize(0); }

    };

}
