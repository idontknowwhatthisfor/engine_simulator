// OpenGL 2.1 renderer: engine groups, ghost block, combustion glow, motion blur, floor, gear meshes
#pragma once
#include "model.hpp"

// ---------------------------------------------------------------------------------------------
// renderer
// ---------------------------------------------------------------------------------------------
struct frameEffects {
    std::array<float, 4> glow{};       // piston crown glow 0..1 per cylinder
    std::array<float, 4> crownY{};     // mesh-space crown top y per cylinder (rest pose)
    std::array<float, 4> flame{};      // flame volume brightness 0..1
    std::array<mat4, 4> flameModel{};  // unit cylinder -> world, spanning crown to deck
    std::vector<std::vector<mat4>> ghostPoses;  // motion blur poses, oldest first
    std::vector<float> ghostAlpha;
    float gridScroll = 0.f;            // road distance in mm, scrolls the floor grid
};

// procedurally built meshes (the gearbox), same vertex layout as the engine groups
struct meshBuf {
    std::vector<float> v;  // px py pz nx ny nz
    std::vector<std::uint32_t> i;
    GLuint vbo = 0, ibo = 0;

    void tri(vec3 a, vec3 b, vec3 c) {
        vec3 n = cross(b - a, c - a);
        float l = length(n);
        n = l > 0.f ? n * (1.f / l) : vec3{0.f, 0.f, 1.f};
        for (vec3 p : {a, b, c}) {
            v.insert(v.end(), {p.x, p.y, p.z, n.x, n.y, n.z});
            i.push_back(static_cast<std::uint32_t>(i.size()));
        }
    }
    void quad(vec3 a, vec3 b, vec3 c, vec3 d) {
        tri(a, b, c);
        tri(a, c, d);
    }
};

// spur gear on the z axis, centred at z = 0, teeth centred at angles 2*pi*k/n
// general meshing phase: gear B (nB) driven by gear A (nA) touching in direction `theta` from A's
// centre: angleB = -angleA * nA / nB + offset puts a tooth of A into a gap of B at the contact
float gearMeshPhase(float theta, int nA, int nB) { return theta + PI + PI / nB + theta * nA / nB; }

meshBuf makeGear(float pitchR, int teeth, float width, float hubR) {
    meshBuf m;
    float h = std::min(4.5f, 0.12f * pitchR), p = 2.f * PI / teeth;
    std::vector<std::pair<float, float>> prof;  // (radius, angle)
    for (int k = 0; k < teeth; k++) {
        float c = k * p;
        prof.push_back({pitchR - h, c - 0.30f * p});
        prof.push_back({pitchR + h, c - 0.15f * p});
        prof.push_back({pitchR + h, c + 0.15f * p});
        prof.push_back({pitchR - h, c + 0.30f * p});
    }
    float zf = width / 2.f, zb = -width / 2.f;
    auto at = [](float r, float a, float z) { return vec3{r * std::cos(a), r * std::sin(a), z}; };
    for (std::size_t j = 0; j < prof.size(); j++) {
        auto [r0, a0] = prof[j];
        auto [r1, a1] = prof[(j + 1) % prof.size()];
        if (j + 1 == prof.size()) a1 += 2.f * PI;
        m.tri(at(hubR, a0, zf), at(r0, a0, zf), at(r1, a1, zf));
        m.tri(at(hubR, a0, zf), at(r1, a1, zf), at(hubR, a1, zf));
        m.tri(at(hubR, a0, zb), at(r1, a1, zb), at(r0, a0, zb));
        m.tri(at(hubR, a0, zb), at(hubR, a1, zb), at(r1, a1, zb));
        m.quad(at(r0, a0, zf), at(r0, a0, zb), at(r1, a1, zb), at(r1, a1, zf));
    }
    return m;
}

// solid cylinder on the z axis from z0 to z1
meshBuf makeCylinder(float r, float z0, float z1, int seg = 32) {
    meshBuf m;
    for (int k = 0; k < seg; k++) {
        float a0 = 2.f * PI * k / seg, a1 = 2.f * PI * (k + 1) / seg;
        vec3 p0{r * std::cos(a0), r * std::sin(a0), 0.f}, p1{r * std::cos(a1), r * std::sin(a1), 0.f};
        m.quad({p0.x, p0.y, z0}, {p1.x, p1.y, z0}, {p1.x, p1.y, z1}, {p0.x, p0.y, z1});
        m.tri({0.f, 0.f, z1}, {p0.x, p0.y, z1}, {p1.x, p1.y, z1});
        m.tri({0.f, 0.f, z0}, {p1.x, p1.y, z0}, {p0.x, p0.y, z0});
    }
    return m;
}

meshBuf makeBox(vec3 lo, vec3 hi) {
    meshBuf m;
    vec3 c[8];
    for (int k = 0; k < 8; k++) c[k] = {k & 1 ? hi.x : lo.x, k & 2 ? hi.y : lo.y, k & 4 ? hi.z : lo.z};
    m.quad(c[0], c[2], c[3], c[1]); m.quad(c[4], c[5], c[7], c[6]);
    m.quad(c[0], c[1], c[5], c[4]); m.quad(c[2], c[6], c[7], c[3]);
    m.quad(c[0], c[4], c[6], c[2]); m.quad(c[1], c[3], c[7], c[5]);
    return m;
}

struct drawItem {
    const meshBuf *mesh;
    mat4 model;
    float rgba[4];
    float roughness, metallic;
    float glow[3] = {0.f, 0.f, 0.f};  // emissive colour over the whole part (hot exhaust), linear
    int additive = 0;                  // 0 lit; light added on top: 1 flat (sparks), 2 soft flame volume
};

mat4 scale(vec3 s) {
    mat4 r = identity();
    r.at(0, 0) = s.x;
    r.at(1, 1) = s.y;
    r.at(2, 2) = s.z;
    return r;
}
const char *VERT_SRC = R"(
#version 120
attribute vec3 aPos;
attribute vec3 aNor;
uniform mat4 uModelView;
uniform mat4 uProj;
varying vec3 vPos;
varying vec3 vNor;
varying float vMeshY;  // mesh-space y, for the piston crown glow and the flame gradient
void main() {
    vec4 p = uModelView * vec4(aPos, 1.0);
    vPos = p.xyz;
    vMeshY = aPos.y;
    // the model matrices are rigid (rotation + translation + axis flips), so no inverse-transpose needed
    vNor = mat3(uModelView) * aNor;
    gl_Position = uProj * p;
}
)";

// Blinn-Phong with the sphere sim's light: from the upper left, towards the viewer
const char *FRAG_SRC = R"(
#version 120
varying vec3 vPos;
varying vec3 vNor;
varying float vMeshY;
uniform vec4 uColor;       // linear base colour + alpha
uniform float uRoughness;
uniform float uMetallic;
uniform float uUnlit;      // 1 = flat colour (floor, grid), 2 = flame volume (additive)
uniform vec3 uLight;       // eye space
uniform vec3 uGlow;        // emissive colour on the piston crown, linear
uniform float uCrownY;     // mesh-space y of the crown top (bores point toward -y)
void main() {
    if (uUnlit > 1.5) {
        // unit cylinder, y = 0 at the crown, 1 at the deck; brighter through the middle, soft at the rim
        vec3 n = normalize(vNor);
        float facing = abs(dot(n, normalize(-vPos)));
        float body = pow(facing, 1.5) * (0.55 + 0.45 * (1.0 - vMeshY));
        gl_FragColor = vec4(uColor.rgb * body, 1.0);
        return;
    }
    if (uUnlit > 0.5) {
        gl_FragColor = uColor;
        return;
    }
    vec3 n = normalize(vNor);
    if (!gl_FrontFacing) n = -n;  // materials are double sided
    vec3 v = normalize(-vPos);
    vec3 h = normalize(uLight + v);
    float diffuse = max(dot(n, uLight), 0.0);
    // fill light from below-right so the dark side of parts still reads
    float fill = max(dot(n, normalize(vec3(0.6, -0.4, 0.5))), 0.0);
    float shininess = mix(90.0, 8.0, uRoughness);
    float spec = pow(max(dot(n, h), 0.0), shininess) * mix(0.55, 0.08, uRoughness);
    vec3 base = uColor.rgb;
    vec3 specCol = mix(vec3(1.0), base, uMetallic);
    vec3 lin = base * (0.18 + 0.85 * diffuse + 0.15 * fill) + specCol * spec;
    // hot crown: strongest on the top face, fading down the ring lands
    float w = clamp(1.0 - (vMeshY - uCrownY) / 14.0, 0.0, 1.0);
    lin += uGlow * w * w;
    gl_FragColor = vec4(pow(lin, vec3(1.0 / 2.2)), uColor.a);
}
)";

GLuint compile(GLenum type, const char *src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[2048];
        glGetShaderInfoLog(s, sizeof log, nullptr, log);
        throw std::runtime_error(std::string("shader compile: ") + log);
    }
    return s;
}

struct renderer {
    GLuint prog = 0;
    GLint aPos = -1, aNor = -1;
    GLint uModelView = -1, uProj = -1, uColor = -1, uRoughness = -1, uMetallic = -1, uUnlit = -1, uLight = -1;
    GLint uGlow = -1, uCrownY = -1;
    GLuint floorVbo = 0, gridVbo = 0, flameVbo = 0;
    int gridVerts = 0, flameVerts = 0;
    float floorY = 0.f, floorCx = 0.f, floorCz = 0.f, floorHalf = 0.f;

    void upload(meshBuf &m) {
        glGenBuffers(1, &m.vbo);
        glBindBuffer(GL_ARRAY_BUFFER, m.vbo);
        glBufferData(GL_ARRAY_BUFFER, m.v.size() * sizeof(float), m.v.data(), GL_STATIC_DRAW);
        glGenBuffers(1, &m.ibo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.ibo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, m.i.size() * sizeof(std::uint32_t), m.i.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    // which way the vehicle faces in world space: along z (the car, crank along the car) or along x
    // (the bikes, crank across the bike), and the sign of "forward" on that axis
    bool travelX = false;
    float travelSign = 1.f;

    // grid lines; the ones across the direction of travel slide back with the road
    void updateGrid(float scroll) {
        float y = floorY + 0.5f, half = floorHalf, cx = floorCx, cz = floorCz;
        const int LINES = 8;
        float spacing = 2.f * half / LINES;
        std::vector<float> grid;
        auto seg = [&](vec3 a, vec3 b) { grid.insert(grid.end(), {a.x, a.y, a.z, 0.f, 1.f, 0.f, b.x, b.y, b.z, 0.f, 1.f, 0.f}); };
        // a point `along` the travel axis and `across` it, from the floor centre
        auto at = [&](float along, float across) {
            return travelX ? vec3{cx + along, y, cz + across} : vec3{cx + across, y, cz + along};
        };
        for (int k = 1; k < LINES; k++) {
            float t = -half + spacing * k;
            seg(at(-half, t), at(half, t));
        }
        float off = std::fmod(scroll, spacing);
        if (off < 0.f) off += spacing;
        for (int k = 0; k <= LINES; k++) {
            float t = -half + spacing * k - travelSign * off;  // the road runs backward under the vehicle
            if (t > -half + 0.5f && t < half - 0.5f) seg(at(t, -half), at(t, half));
        }
        gridVerts = static_cast<int>(grid.size() / 6);
        glBindBuffer(GL_ARRAY_BUFFER, gridVbo);
        glBufferData(GL_ARRAY_BUFFER, grid.size() * sizeof(float), grid.data(), GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void init(engineModel &e) {
        prog = glCreateProgram();
        glAttachShader(prog, compile(GL_VERTEX_SHADER, VERT_SRC));
        glAttachShader(prog, compile(GL_FRAGMENT_SHADER, FRAG_SRC));
#ifdef __APPLE__
        // macOS legacy GL silently skips draws unless generic attribute 0 is an enabled array
        aPos = 0;
        aNor = 1;
#else
        // keep our attributes off index 0, which some drivers alias with SFML's fixed-function gl_Vertex
        aPos = 1;
        aNor = 2;
#endif
        glBindAttribLocation(prog, aPos, "aPos");
        glBindAttribLocation(prog, aNor, "aNor");
        glLinkProgram(prog);
        GLint ok = 0;
        glGetProgramiv(prog, GL_LINK_STATUS, &ok);
        if (!ok) {
            char log[2048];
            glGetProgramInfoLog(prog, sizeof log, nullptr, log);
            throw std::runtime_error(std::string("shader link: ") + log);
        }
        uModelView = glGetUniformLocation(prog, "uModelView");
        uProj = glGetUniformLocation(prog, "uProj");
        uColor = glGetUniformLocation(prog, "uColor");
        uRoughness = glGetUniformLocation(prog, "uRoughness");
        uMetallic = glGetUniformLocation(prog, "uMetallic");
        uUnlit = glGetUniformLocation(prog, "uUnlit");
        uLight = glGetUniformLocation(prog, "uLight");
        uGlow = glGetUniformLocation(prog, "uGlow");
        uCrownY = glGetUniformLocation(prog, "uCrownY");

        // unit cylinder (radius 1, y 0..1) with a cap at y = 1, for the combustion flame
        std::vector<float> flame;
        const int SEG = 40;
        for (int k = 0; k < SEG; k++) {
            float a0 = 2.f * PI * k / SEG, a1 = 2.f * PI * (k + 1) / SEG;
            float c0 = std::cos(a0), s0 = std::sin(a0), c1 = std::cos(a1), s1 = std::sin(a1);
            float side[6][6] = {{c0, 0, s0, c0, 0, s0}, {c1, 0, s1, c1, 0, s1}, {c1, 1, s1, c1, 0, s1},
                                {c0, 0, s0, c0, 0, s0}, {c1, 1, s1, c1, 0, s1}, {c0, 1, s0, c0, 0, s0}};
            float cap[3][6] = {{0, 1, 0, 0, 1, 0}, {c0, 1, s0, 0, 1, 0}, {c1, 1, s1, 0, 1, 0}};
            for (auto &v : side) flame.insert(flame.end(), v, v + 6);
            for (auto &v : cap) flame.insert(flame.end(), v, v + 6);
        }
        flameVerts = static_cast<int>(flame.size() / 6);
        glGenBuffers(1, &flameVbo);
        glBindBuffer(GL_ARRAY_BUFFER, flameVbo);
        glBufferData(GL_ARRAY_BUFFER, flame.size() * sizeof(float), flame.data(), GL_STATIC_DRAW);

        for (auto &g : e.groups) {
            glGenBuffers(1, &g.vbo);
            glBindBuffer(GL_ARRAY_BUFFER, g.vbo);
            glBufferData(GL_ARRAY_BUFFER, g.verts.size() * sizeof(float), g.verts.data(), GL_STATIC_DRAW);
            glGenBuffers(1, &g.ibo);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g.ibo);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, g.idx.size() * sizeof(std::uint32_t), g.idx.data(), GL_STATIC_DRAW);
        }

        // floor slightly under the engine, a square like the sphere sim's box floor
        float y = e.lo.y - 2.f;
        float cx = (e.lo.x + e.hi.x) / 2.f, cz = (e.lo.z + e.hi.z) / 2.f;
        float half = 1.25f * std::max(e.hi.x - e.lo.x, e.hi.z - e.lo.z) / 2.f;
        floorY = y; floorCx = cx; floorCz = cz; floorHalf = half;
        std::vector<float> fl = {
            cx - half, y, cz - half, 0, 1, 0,  cx + half, y, cz - half, 0, 1, 0,
            cx + half, y, cz + half, 0, 1, 0,  cx - half, y, cz + half, 0, 1, 0,
        };
        std::vector<float> grid;
        const int LINES = 8;
        for (int k = 1; k < LINES; k++) {
            float t = -half + 2.f * half * k / LINES;
            float seg[4][3] = {{cx + t, y + 0.5f, cz - half}, {cx + t, y + 0.5f, cz + half},
                               {cx - half, y + 0.5f, cz + t}, {cx + half, y + 0.5f, cz + t}};
            for (auto &p : seg) grid.insert(grid.end(), {p[0], p[1], p[2], 0.f, 1.f, 0.f});
        }
        gridVerts = static_cast<int>(grid.size() / 6);
        glGenBuffers(1, &floorVbo);
        glBindBuffer(GL_ARRAY_BUFFER, floorVbo);
        glBufferData(GL_ARRAY_BUFFER, fl.size() * sizeof(float), fl.data(), GL_STATIC_DRAW);
        glGenBuffers(1, &gridVbo);
        glBindBuffer(GL_ARRAY_BUFFER, gridVbo);
        glBufferData(GL_ARRAY_BUFFER, grid.size() * sizeof(float), grid.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        updateGrid(0.f);
    }

    void bindVerts(GLuint vbo) {
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexAttribPointer(aPos, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void *>(0));
        glVertexAttribPointer(aNor, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void *>(3 * sizeof(float)));
    }

    void setColor(float r, float g, float b, float a) { glUniform4f(uColor, r, g, b, a); }

    // ghost: 0 = translucent block, 1 = block hidden
    void draw(const engineModel &e, const std::vector<mat4> &modelOf, const mat4 &view, const mat4 &proj, int ghost,
              const frameEffects &fx, const std::vector<drawItem> &extras = {}) {
        updateGrid(fx.gridScroll);
        glUseProgram(prog);
        glEnableVertexAttribArray(aPos);
        glEnableVertexAttribArray(aNor);
        glUniformMatrix4fv(uProj, 1, GL_FALSE, proj.m.data());
        vec3 light = normalize({-0.45f, 0.55f, 0.7f});
        glUniform3f(uLight, light.x, light.y, light.z);

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glDepthMask(GL_TRUE);
        glDisable(GL_CULL_FACE);
        glDisable(GL_BLEND);

        // floor and grid, unlit, same colours as the sphere sim
        glUniformMatrix4fv(uModelView, 1, GL_FALSE, view.m.data());
        glUniform1f(uUnlit, 1.f);
        bindVerts(floorVbo);
        setColor(35 / 255.f, 38 / 255.f, 48 / 255.f, 1.f);
        glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
        bindVerts(gridVbo);
        setColor(55 / 255.f, 60 / 255.f, 75 / 255.f, 1.f);
        glDrawArrays(GL_LINES, 0, gridVerts);
        glUniform1f(uUnlit, 0.f);

        glUniform3f(uGlow, 0.f, 0.f, 0.f);
        glUniform1f(uCrownY, 0.f);
        auto drawGroup = [&](std::size_t k, const mat4 &model, float alpha) {
            const group &g = e.groups[k];
            const material &m = e.mats[g.mat];
            mat4 mv = view * model;
            glUniformMatrix4fv(uModelView, 1, GL_FALSE, mv.m.data());
            setColor(m.rgba[0], m.rgba[1], m.rgba[2], alpha >= 0.f ? alpha : (m.blend ? m.rgba[3] : 1.f));
            bool hot = g.kind == PISTON && g.cyl >= 0 && fx.glow[g.cyl] > 0.f;
            if (hot) {
                float t = fx.glow[g.cyl];
                // dull red -> orange -> yellow-white as the gas gets hotter
                glUniform3f(uGlow, 2.2f * t, (0.25f + 1.1f * t) * t, 0.35f * t * t * t);
                glUniform1f(uCrownY, fx.crownY[g.cyl]);
            }
            glUniform1f(uRoughness, m.roughness);
            glUniform1f(uMetallic, m.metallic);
            bindVerts(g.vbo);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g.ibo);
            glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(g.idx.size()), GL_UNSIGNED_INT, nullptr);
            if (hot) glUniform3f(uGlow, 0.f, 0.f, 0.f);
        };

        // opaque parts first; the block's opaque bits (freeze plugs) hide with the block
        for (std::size_t k = 0; k < e.groups.size(); k++)
            if (!e.mats[e.groups[k].mat].blend && (ghost == 0 || e.groups[k].kind != BLOCK)) drawGroup(k, modelOf[k], -1.f);

        // extra meshes (gearbox): opaque ones now, see-through ones with the block
        auto drawItemNow = [&](const drawItem &it) {
            mat4 mv = view * it.model;
            glUniformMatrix4fv(uModelView, 1, GL_FALSE, mv.m.data());
            setColor(it.rgba[0], it.rgba[1], it.rgba[2], it.rgba[3]);
            glUniform1f(uRoughness, it.roughness);
            glUniform1f(uMetallic, it.metallic);
            bool hot = it.glow[0] > 0.f || it.glow[1] > 0.f || it.glow[2] > 0.f;
            if (hot) {
                glUniform3f(uGlow, it.glow[0], it.glow[1], it.glow[2]);
                glUniform1f(uCrownY, 1e9f);  // the crown fade is then 1 everywhere: the whole part glows
            }
            bindVerts(it.mesh->vbo);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, it.mesh->ibo);
            glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(it.mesh->i.size()), GL_UNSIGNED_INT, nullptr);
            if (hot) {
                glUniform3f(uGlow, 0.f, 0.f, 0.f);
                glUniform1f(uCrownY, 0.f);
            }
        };
        for (const drawItem &it : extras)
            if (it.rgba[3] >= 1.f && !it.additive) drawItemNow(it);

        // motion blur: faded copies of the moving parts at earlier poses within this frame
        if (!fx.ghostPoses.empty()) {
            glEnable(GL_BLEND);
            glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ZERO, GL_ONE);  // keep framebuffer alpha at 1
            glDepthMask(GL_FALSE);
            for (std::size_t n = 0; n < fx.ghostPoses.size(); n++)
                for (std::size_t k = 0; k < e.groups.size(); k++) {
                    partKind kind = e.groups[k].kind;
                    if (kind == CRANK || kind == PISTON || kind == ROD) drawGroup(k, fx.ghostPoses[n][k], fx.ghostAlpha[n]);
                }
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
        }

        // combustion flame in the space between the crown and the deck, additive
        bool anyFlame = false;
        for (float f : fx.flame) anyFlame |= f > 0.f;
        if (anyFlame) {
            glEnable(GL_BLEND);
            glBlendFuncSeparate(GL_ONE, GL_ONE, GL_ZERO, GL_ONE);
            glDepthMask(GL_FALSE);
            glUniform1f(uUnlit, 2.f);
            bindVerts(flameVbo);
            for (std::size_t c = 0; c < fx.flame.size(); c++) {
                float t = fx.flame[c];
                if (t <= 0.f) continue;
                mat4 mv = view * fx.flameModel[c];
                glUniformMatrix4fv(uModelView, 1, GL_FALSE, mv.m.data());
                setColor(0.9f * t, (0.28f + 0.45f * t) * t, 0.06f * t * t, 1.f);
                glDrawArrays(GL_TRIANGLES, 0, flameVerts);
            }
            glUniform1f(uUnlit, 0.f);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
        }

        // exhaust fire and sparks: light added on top, tested against the solid parts but not hiding anything
        bool anyAdd = false;
        for (const drawItem &it : extras) anyAdd |= it.additive != 0;
        if (anyAdd) {
            glEnable(GL_BLEND);
            glBlendFuncSeparate(GL_ONE, GL_ONE, GL_ZERO, GL_ONE);
            glDepthMask(GL_FALSE);
            for (const drawItem &it : extras) {
                if (!it.additive) continue;
                glUniform1f(uUnlit, it.additive == 2 ? 2.f : 1.f);
                mat4 mv = view * it.model;
                glUniformMatrix4fv(uModelView, 1, GL_FALSE, mv.m.data());
                setColor(it.rgba[0], it.rgba[1], it.rgba[2], 1.f);
                bindVerts(it.mesh->vbo);
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, it.mesh->ibo);
                glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(it.mesh->i.size()), GL_UNSIGNED_INT, nullptr);
            }
            glUniform1f(uUnlit, 0.f);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
        }

        // then the see-through block: no depth writes so the internals stay visible, back faces culled
        // so each wall is blended once
        if (ghost == 0) {
            glEnable(GL_BLEND);
            glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ZERO, GL_ONE);  // keep framebuffer alpha at 1
            glDepthMask(GL_FALSE);
            glEnable(GL_CULL_FACE);
            glCullFace(GL_BACK);
            for (std::size_t k = 0; k < e.groups.size(); k++)
                if (e.mats[e.groups[k].mat].blend) drawGroup(k, modelOf[k], -1.f);
            for (const drawItem &it : extras)
                if (it.rgba[3] < 1.f && !it.additive) drawItemNow(it);
            glDisable(GL_CULL_FACE);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
        }

        // leave GL tidy for SFML's text drawing
        glDisableVertexAttribArray(aPos);
        glDisableVertexAttribArray(aNor);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glUseProgram(0);
    }
};

