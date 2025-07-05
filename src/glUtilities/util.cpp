#include "util.h"
#include "glad/glad.h"
#include <stdio.h>

namespace gl {
    void GLClearError() {
        while(glGetError() != GL_NO_ERROR);
    }

    bool GLLogCall(const char *function, const char *file, int line) {
        while(GLenum error = glGetError()) {
            printf("[%d:%s %d %s]\n", error, file, line, function);
            return false;
        }
        return true;
    }

}
