#include "VertexArray.h"
#include "VertexBufferLayout.h"
#include "glad/glad.h"

namespace gl {
    VertexArray::VertexArray() {
        GLCALL(glGenVertexArrays(1, &m_id));
        bind();
    }

    VertexArray::~VertexArray() {
        GLCALL(glDeleteVertexArrays(1, &m_id));
    }

    void VertexArray::applyBufferLayout(const VertexBufferLayout &layout) {
        const auto &elements = layout.elements();
        u64 offset = 0;

        for(u32 i = 0; i < elements.size(); i++) {
            const auto &element = elements[i];
            glVertexAttribPointer(i, element.count, element.type, element.normalized, layout.stride(), (void*)offset);
            glEnableVertexAttribArray(i);
            offset += element.count*VertexBufferElement::type_size(element.type);
        }
    }

    void VertexArray::bind() const {
        GLCALL(glBindVertexArray(m_id));
    };

    void VertexArray::unbind() const {
        GLCALL(glBindVertexArray(0));
    };

}
