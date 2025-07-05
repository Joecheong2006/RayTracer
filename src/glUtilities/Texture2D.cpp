#include "Texture2D.h"
#include "glad/glad.h"
#include "stb_image/stb_image.h"
#include "util.h"

namespace gl {
    Texture2D::Construct Texture2D::load(const std::string &path) {
        Construct con;
        con.data = stbi_load(path.c_str(), &con.width, &con.height, &con.bpp, 0);
        con.format = con.bpp == 4 ? GL_RGBA : GL_RGB;
        return con;
    }

    Texture2D::Texture2D(const Construct &con)
        : con(con)
    {
        GLCALL(glGenTextures(1, &m_id));
        GLCALL(glBindTexture(GL_TEXTURE_2D, m_id));
        GLCALL(glTexImage2D(GL_TEXTURE_2D, 0, con.internal != 0 ? con.internal : con.format,
                    con.width, con.height, 0, con.format, con.type, con.data));

        if (con.mipmap)
            GLCALL(glGenerateMipmap(GL_TEXTURE_2D));

        GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, con.wrapStyle));	
        GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, con.wrapStyle));

        GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, con.style));
        GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, con.style));

        // NOTE: create an own method for setting the border color
        float color[4] = { 1.0, 1.0, 1.0, 1.0 };
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, color);
    }

    Texture2D::~Texture2D()
    {
        GLCALL(glDeleteTextures(1, &m_id));
    }

    void Texture2D::bind(u32 slot) const
    {
        GLCALL(glActiveTexture(GL_TEXTURE0 + slot));
        GLCALL(glBindTexture(GL_TEXTURE_2D, m_id));
    }

    void Texture2D::unbind() const
    {
        GLCALL(glBindTexture(GL_TEXTURE_2D, 0));
    }

}
