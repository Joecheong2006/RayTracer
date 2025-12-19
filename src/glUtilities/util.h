#pragma once
#include "../util.h"

#define GLCALL(x) gl::GLClearError();\
    x;\
    ASSERT(gl::GLLogCall(#x, __FILE__, __LINE__))\

namespace gl {
    void GLClearError();
    bool GLLogCall(const char *function, const char *file, int line);

}
