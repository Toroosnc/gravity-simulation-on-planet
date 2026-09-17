#include <epoxy/gl.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

#include "body.hpp"

constexpr double G = 1.0;
constexpr double SOFTENING = 0.08;
constexpr double TIME_STEP = 0.0002;
constexpr int PHYSICS_SUBSTEPS = 6;
constexpr int WINDOW_WIDTH = 1500;
constexpr int WINDOW_HEIGHT = 900;
constexpr float PI = 3.14159265359f;

struct Mat4 {
    float value[16]{};
};

Mat4 identity_matrix() {
    Mat4 matrix{};
    matrix.value[0] = matrix.value[5] = matrix.value[10] = matrix.value[15] = 1.0f;
    return matrix;
}

Mat4 multiply(const Mat4& left, const Mat4& right) {
    Mat4 result{};
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            for (int index = 0; index < 4; ++index) {
                result.value[column * 4 + row] += left.value[index * 4 + row] * right.value[column * 4 + index];
            }
        }
    }
    return result;
}

Mat4 perspective(float field_of_view, float aspect, float near_plane, float far_plane) {
    const float tangent = std::tan(field_of_view * 0.5f);
    Mat4 result{};
    result.value[0] = 1.0f / (aspect * tangent);
    result.value[5] = 1.0f / tangent;
    result.value[10] = -(far_plane + near_plane) / (far_plane - near_plane);
    result.value[11] = -1.0f;
    result.value[14] = -(2.0f * far_plane * near_plane) / (far_plane - near_plane);
    return result;
}

Mat4 look_at(float eye_x, float eye_y, float eye_z) {
    const float length = std::sqrt(eye_x * eye_x + eye_y * eye_y + eye_z * eye_z);
    const float forward_x = -eye_x / length;
    const float forward_y = -eye_y / length;
    const float forward_z = -eye_z / length;
    const float right_length = std::sqrt(forward_x * forward_x + forward_z * forward_z);
    const float right_x = forward_z / right_length;
    const float right_z = -forward_x / right_length;
    const float up_x = -right_z * forward_y;
    const float up_y = right_z * forward_x - right_x * forward_z;
    const float up_z = right_x * forward_y;

    Mat4 result{};
    result.value[0] = right_x;
    result.value[1] = up_x;
    result.value[2] = -forward_x;
    result.value[4] = up_y * 0.0f;
    result.value[5] = up_y;
    result.value[6] = -forward_y;
    result.value[8] = right_z;
    result.value[9] = up_z;
    result.value[10] = -forward_z;
    result.value[12] = 0.0f;
    result.value[13] = 0.0f;
    result.value[14] = -length;
    result.value[15] = 1.0f;
    return result;
}

std::vector<Vec2> accelerations(const std::vector<Body>& bodies) {
    std::vector<Vec2> acceleration(bodies.size());
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        for (std::size_t j = i + 1; j < bodies.size(); ++j) {
            const bool moon_i = bodies[i].name == "Moon";
            const bool moon_j = bodies[j].name == "Moon";
            if (moon_i && bodies[j].name != "Earth") continue;
            if (moon_j && bodies[i].name != "Earth") continue;

            const Vec2 difference = bodies[j].position - bodies[i].position;
            const double distance_squared = difference.x * difference.x + difference.y * difference.y;
            const double distance = std::sqrt(distance_squared + SOFTENING * SOFTENING);
            const Vec2 direction = difference * (1.0 / distance);
            const double force = G * bodies[i].mass * bodies[j].mass /
                (distance * distance + SOFTENING * SOFTENING);
            acceleration[i] += direction * (force / bodies[i].mass);
            acceleration[j] += direction * (-force / bodies[j].mass);
        }
    }
    return acceleration;
}

void update(std::vector<Body>& bodies) {
    const std::vector<Vec2> old_acceleration = accelerations(bodies);
    const double time_step_squared = TIME_STEP * TIME_STEP;
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        bodies[i].position += bodies[i].velocity * TIME_STEP + old_acceleration[i] * (0.5 * time_step_squared);
    }

    const std::vector<Vec2> new_acceleration = accelerations(bodies);
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        bodies[i].velocity += (old_acceleration[i] + new_acceleration[i]) * (0.5 * TIME_STEP);
    }

    std::size_t earth_index = std::size_t(-1);
    std::size_t moon_index = std::size_t(-1);
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i].name == "Earth") earth_index = i;
        if (bodies[i].name == "Moon") moon_index = i;
    }
    if (earth_index != std::size_t(-1) && moon_index != std::size_t(-1)) {
        Vec2 moon_offset = bodies[moon_index].position - bodies[earth_index].position;
        const double moon_distance = std::sqrt(moon_offset.x * moon_offset.x + moon_offset.y * moon_offset.y);
        const double desired_distance = 0.9;
        if (moon_distance > 0.0001) {
            moon_offset = moon_offset * (desired_distance / moon_distance);
            bodies[moon_index].position = bodies[earth_index].position + moon_offset;
        }

        const Vec2 tangent = {-moon_offset.y, moon_offset.x};
        const double tangent_length = std::sqrt(tangent.x * tangent.x + tangent.y * tangent.y);
        const Vec2 orbit_velocity = tangent * (1.05 / (tangent_length > 0.0 ? tangent_length : 1.0));
        bodies[moon_index].velocity = orbit_velocity + bodies[earth_index].velocity * 0.25;
    }
}

std::vector<Body> initial_bodies() {
    return {
        {"Sun", {0.0, 0.0}, {0.0, 0.0}, 5000.0, 32, "#ffb703", 0.0},
        {"Earth", {8.5, 0.0}, {0.0, 24.26}, 1.0, 8, "#4cc9f0", 8.5},
        {"Moon", {9.2, 0.0}, {0.0, 25.31}, 0.0123, 3, "#c6ccd4", 0.7},
        {"Mars", {-18.0, 0.0}, {0.0, -16.67}, 0.107, 6, "#ef8354", 18.0},
    };
}

GLuint make_shader(GLenum type, const char* source) {
    const GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    GLint success = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char log[1024]{};
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        std::cerr << "Shader error: " << log << '\n';
    }
    return shader;
}

GLuint make_program() {
    const char* vertex_source = R"glsl(
        #version 330 core
        layout(location = 0) in vec3 position;
        layout(location = 1) in vec3 normal;
        uniform mat4 uMvp;
        uniform mat4 uModel;
        out vec3 surface_normal;
        out vec3 surface_position;
        void main() {
            surface_normal = mat3(uModel) * normal;
            surface_position = vec3(uModel * vec4(position, 1.0));
            gl_Position = uMvp * vec4(position, 1.0);
        }
    )glsl";
    const char* fragment_source = R"glsl(
        #version 330 core
        in vec3 surface_normal;
        in vec3 surface_position;
        uniform vec3 uColor;
        uniform vec3 uLightPosition;
        uniform float uEmission;
        out vec4 pixel;
        void main() {
            vec3 normal = normalize(surface_normal);
            vec3 light_direction = normalize(uLightPosition - surface_position);
            vec3 view_direction = normalize(vec3(0.0, 0.0, 1.0) - surface_position);
            vec3 reflect_direction = reflect(-light_direction, normal);

            float diffuse = max(dot(normal, light_direction), 0.0);
            float specular = pow(max(dot(reflect_direction, view_direction), 0.0), 24.0);
            float fresnel = pow(1.0 - max(dot(normal, view_direction), 0.0), 2.6);
            float atmospheric = pow(1.0 - max(dot(normal, vec3(0.0, 0.0, 1.0)), 0.0), 2.0);

            vec3 warm_highlight = vec3(1.0, 0.78, 0.38) * specular * 1.1;
            vec3 warm_glow = vec3(1.0, 0.72, 0.2) * atmospheric * 0.9;
            vec3 cool_rim = vec3(0.35, 0.52, 0.9) * fresnel * 0.5;
            vec3 base = uColor * (0.24 + diffuse * 0.76);

            vec3 final_color = base + warm_highlight + warm_glow + cool_rim + uColor * (uEmission * 1.2);
            pixel = vec4(final_color, 1.0);
        }
    )glsl";
    const GLuint vertex_shader = make_shader(GL_VERTEX_SHADER, vertex_source);
    const GLuint fragment_shader = make_shader(GL_FRAGMENT_SHADER, fragment_source);
    const GLuint program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);
    return program;
}

GLuint make_orbit_program() {
    const char* vertex_source = R"glsl(
        #version 330 core
        layout(location = 0) in vec3 position;
        uniform mat4 uMvp;
        void main() {
            gl_Position = uMvp * vec4(position, 1.0);
        }
    )glsl";
    const char* fragment_source = R"glsl(
        #version 330 core
        uniform vec3 uColor;
        out vec4 pixel;
        void main() {
            pixel = vec4(uColor, 0.42);
        }
    )glsl";
    const GLuint vertex_shader = make_shader(GL_VERTEX_SHADER, vertex_source);
    const GLuint fragment_shader = make_shader(GL_FRAGMENT_SHADER, fragment_source);
    const GLuint program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);
    return program;
}

void make_sphere(std::vector<float>& vertices, std::vector<unsigned int>& indices) {
    constexpr int stacks = 24;
    constexpr int slices = 32;
    for (int stack = 0; stack <= stacks; ++stack) {
        const float phi = PI * stack / stacks;
        for (int slice = 0; slice <= slices; ++slice) {
            const float theta = 2.0f * PI * slice / slices;
            const float x = std::sin(phi) * std::cos(theta);
            const float y = std::cos(phi);
            const float z = std::sin(phi) * std::sin(theta);
            vertices.insert(vertices.end(), {x, y, z, x, y, z});
        }
    }
    for (int stack = 0; stack < stacks; ++stack) {
        for (int slice = 0; slice < slices; ++slice) {
            const unsigned int first = stack * (slices + 1) + slice;
            const unsigned int second = first + slices + 1;
            indices.insert(indices.end(), {first, second, first + 1, second, second + 1, first + 1});
        }
    }
}

void hex_to_rgb(const char* hex, float& red, float& green, float& blue) {
    unsigned int value = 0;
    const char* cursor = hex;
    if (cursor[0] == '#') {
        ++cursor;
    }
    std::sscanf(cursor, "%x", &value);
    red = static_cast<float>((value >> 16) & 0xFF) / 255.0f;
    green = static_cast<float>((value >> 8) & 0xFF) / 255.0f;
    blue = static_cast<float>(value & 0xFF) / 255.0f;
}

void toggle_fullscreen(GLFWwindow* window, bool fullscreen, int& saved_x, int& saved_y) {
    GLFWmonitor* monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);
    if (fullscreen) {
        glfwGetWindowPos(window, &saved_x, &saved_y);
        glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
    } else {
        glfwSetWindowMonitor(window, nullptr, saved_x, saved_y, WINDOW_WIDTH, WINDOW_HEIGHT, 0);
    }
}

int main() {
    if (!glfwInit()) {
        std::cerr << "GLFW gagal diinisialisasi.\n";
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Gravity Simulation 3D", nullptr, nullptr);
    if (window == nullptr) {
        std::cerr << "Window OpenGL gagal dibuat.\n";
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    const GLuint program = make_program();
    const GLuint orbit_program = make_orbit_program();
    std::vector<float> sphere_vertices;
    std::vector<unsigned int> sphere_indices;
    make_sphere(sphere_vertices, sphere_indices);
    GLuint sphere_vao = 0;
    GLuint sphere_vbo = 0;
    GLuint sphere_ebo = 0;
    glGenVertexArrays(1, &sphere_vao);
    glGenBuffers(1, &sphere_vbo);
    glGenBuffers(1, &sphere_ebo);
    glBindVertexArray(sphere_vao);
    glBindBuffer(GL_ARRAY_BUFFER, sphere_vbo);
    glBufferData(GL_ARRAY_BUFFER, sphere_vertices.size() * sizeof(float), sphere_vertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sphere_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sphere_indices.size() * sizeof(unsigned int), sphere_indices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    std::vector<float> orbit_vertices;
    constexpr int orbit_segments = 160;
    for (int segment = 0; segment <= orbit_segments; ++segment) {
        const float angle = 2.0f * PI * static_cast<float>(segment) / orbit_segments;
        orbit_vertices.insert(orbit_vertices.end(), {
            std::cos(angle), 0.0f, std::sin(angle)
        });
    }
    GLuint orbit_vao = 0;
    GLuint orbit_vbo = 0;
    glGenVertexArrays(1, &orbit_vao);
    glGenBuffers(1, &orbit_vbo);
    glBindVertexArray(orbit_vao);
    glBindBuffer(GL_ARRAY_BUFFER, orbit_vbo);
    glBufferData(GL_ARRAY_BUFFER, orbit_vertices.size() * sizeof(float), orbit_vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);

    std::vector<Body> bodies = initial_bodies();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LINE_SMOOTH);
    const GLint mvp_location = glGetUniformLocation(program, "uMvp");
    const GLint model_location = glGetUniformLocation(program, "uModel");
    const GLint color_location = glGetUniformLocation(program, "uColor");
    const GLint light_location = glGetUniformLocation(program, "uLightPosition");
    const GLint emission_location = glGetUniformLocation(program, "uEmission");
    const GLint orbit_mvp_location = glGetUniformLocation(orbit_program, "uMvp");
    const GLint orbit_color_location = glGetUniformLocation(orbit_program, "uColor");
    const Mat4 identity = identity_matrix();
    bool paused = false;
    bool fullscreen = true;
    int saved_x = 100;
    int saved_y = 100;
    toggle_fullscreen(window, true, saved_x, saved_y);
    bool previous_space = false;
    bool previous_reset = false;
    bool previous_f11 = false;
    float camera_angle = 0.9f;
    float camera_height = 24.0f;
    float camera_distance = 65.0f;

    while (!glfwWindowShouldClose(window)) {
        const bool space = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
        const bool reset = glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS;
        const bool f11 = glfwGetKey(window, GLFW_KEY_F11) == GLFW_PRESS;
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) break;
        if (space && !previous_space) paused = !paused;
        if (reset && !previous_reset) bodies = initial_bodies();
        if (f11 && !previous_f11) {
            fullscreen = !fullscreen;
            toggle_fullscreen(window, fullscreen, saved_x, saved_y);
        }
        if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) camera_angle -= 0.025f;
        if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) camera_angle += 0.025f;
        if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) camera_height += 0.3f;
        if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) camera_height -= 0.3f;
        if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) camera_distance -= 0.6f;
        if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS) camera_distance += 0.6f;
        camera_height = std::fmax(4.0f, std::fmin(camera_height, 70.0f));
        camera_distance = std::fmax(25.0f, std::fmin(camera_distance, 130.0f));
        previous_space = space;
        previous_reset = reset;
        previous_f11 = f11;
        if (!paused) {
            for (int step = 0; step < PHYSICS_SUBSTEPS; ++step) update(bodies);
        }

        int width = 1;
        int height = 1;
        glfwGetFramebufferSize(window, &width, &height);
        glViewport(0, 0, width, height);
        glClearColor(0.008f, 0.012f, 0.03f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glUseProgram(program);
        const float camera_x = std::sin(camera_angle) * camera_distance;
        const float camera_y = camera_height;
        const float camera_z = std::cos(camera_angle) * camera_distance;
        const Mat4 view_projection = multiply(
            perspective(0.72f, static_cast<float>(width) / height, 0.1f, 220.0f),
            look_at(camera_x, camera_y, camera_z));

        glUseProgram(orbit_program);
        glBindVertexArray(orbit_vao);
        glLineWidth(1.2f);
        for (std::size_t i = 1; i < bodies.size(); ++i) {
            const bool moon = bodies[i].name == "Moon";
            const float orbit_radius = moon ? 0.9f : static_cast<float>(bodies[i].orbit_radius);
            if (orbit_radius <= 0.0f) continue;

            Mat4 orbit_model = identity;
            orbit_model.value[0] = orbit_radius;
            orbit_model.value[10] = orbit_radius;
            if (moon) {
                orbit_model.value[12] = static_cast<float>(bodies[1].position.x);
                orbit_model.value[14] = static_cast<float>(bodies[1].position.y);
            }
            const Mat4 orbit_mvp = multiply(view_projection, orbit_model);
            glUniformMatrix4fv(orbit_mvp_location, 1, GL_FALSE, orbit_mvp.value);
            glUniform3f(orbit_color_location, moon ? 0.42f : 0.06f, moon ? 0.25f : 0.24f, moon ? 0.05f : 0.42f);
            glDrawArrays(GL_LINE_STRIP, 0, orbit_segments + 1);
        }

        glUseProgram(program);
        glUniform3f(light_location, 0.0f, 0.0f, 0.0f);
        glUniform1f(emission_location, 0.0f);
        glUniform3f(color_location, 0.08f, 0.15f, 0.3f);
        glUniformMatrix4fv(mvp_location, 1, GL_FALSE, view_projection.value);
        glUniformMatrix4fv(model_location, 1, GL_FALSE, identity.value);

        glBindVertexArray(sphere_vao);
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            Mat4 model = identity;
            model.value[12] = static_cast<float>(bodies[i].position.x);
            model.value[14] = static_cast<float>(bodies[i].position.y);
            const float scale = i == 0 ? 1.35f : static_cast<float>(bodies[i].radius) * 0.07f;
            const float moon_scale = i == 4 ? 0.6f : 1.0f;
            model.value[0] = model.value[5] = model.value[10] = scale * moon_scale;
            const Mat4 mvp = multiply(view_projection, model);
            glUniformMatrix4fv(mvp_location, 1, GL_FALSE, mvp.value);
            glUniformMatrix4fv(model_location, 1, GL_FALSE, model.value);

            float red = 1.0f;
            float green = 1.0f;
            float blue = 1.0f;
            hex_to_rgb(bodies[i].color, red, green, blue);
            glUniform3f(color_location, red, green, blue);
            glUniform1f(emission_location, i == 0 ? 0.9f : 0.04f);
            glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(sphere_indices.size()), GL_UNSIGNED_INT, nullptr);

        }
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &sphere_vao);
    glDeleteBuffers(1, &sphere_vbo);
    glDeleteBuffers(1, &sphere_ebo);
    glDeleteProgram(program);
    glDeleteVertexArrays(1, &orbit_vao);
    glDeleteBuffers(1, &orbit_vbo);
    glDeleteProgram(orbit_program);
    glfwDestroyWindow(window);
    glfwTerminate();
}
