#pragma once

#ifdef WIN32
#define ASSERT(x) if(!(x)) __debugbreak()
#else
#define ASSERT(x) if(!(x)) __builtin_trap()
#endif

#define GLCALL(x) gl::GLClearError();\
    x;\
    ASSERT(gl::GLLogCall(#x, __FILE__, __LINE__))\

namespace gl {
    void GLClearError();
    bool GLLogCall(const char *function, const char *file, int line);

}
