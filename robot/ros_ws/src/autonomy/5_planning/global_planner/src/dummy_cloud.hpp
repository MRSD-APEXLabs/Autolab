#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <cstring>
#include <cmath>
#include <random>
#include <vector>

// ---------------------------------------------------------------------------
// Coordinate frame: EEF (end-effector) origin
//   +X  = wellplate side
//   +Y  = liquid handler side
//   -Y  = shaker module side
//   +Z  = up
//   Table surface sits at z = -0.05 m (EEF hovers 5 cm above it)
// ---------------------------------------------------------------------------

namespace dummy_cloud
{

// ── Noise ──────────────────────────────────────────────────────────────────
static std::default_random_engine rng_(42);

inline float gauss(float sigma)
{
    std::normal_distribution<float> d(0.f, sigma);
    return d(rng_);
}

// ── Raw point append ───────────────────────────────────────────────────────
inline void push_point(std::vector<uint8_t>& buf, float x, float y, float z,
                       float noise_sigma = 0.003f)
{
    x += gauss(noise_sigma);
    y += gauss(noise_sigma);
    z += gauss(noise_sigma);
    uint8_t bx[4], by[4], bz[4];
    std::memcpy(bx, &x, 4);
    std::memcpy(by, &y, 4);
    std::memcpy(bz, &z, 4);
    buf.insert(buf.end(), bx, bx + 4);
    buf.insert(buf.end(), by, by + 4);
    buf.insert(buf.end(), bz, bz + 4);
}

// ── Geometry helpers ───────────────────────────────────────────────────────

// Flat rectangular surface sampled on a grid
// x in [cx-hw, cx+hw], y in [cy-hd, cy+hd], z = fixed
inline void sample_surface(std::vector<uint8_t>& buf,
                            float cx, float cy, float z,
                            float half_w, float half_d,
                            int nx, int ny,
                            float noise = 0.003f)
{
    for (int i = 0; i < nx; ++i)
        for (int j = 0; j < ny; ++j)
        {
            float x = cx - half_w + (2.f * half_w / (nx - 1)) * i;
            float y = cy - half_d + (2.f * half_d / (ny - 1)) * j;
            push_point(buf, x, y, z, noise);
        }
}

// Vertical rectangular face (parallel to XZ plane, constant y)
inline void sample_face_xz(std::vector<uint8_t>& buf,
                             float cx, float y, float cz,
                             float half_w, float half_h,
                             int nx, int nz,
                             float noise = 0.003f)
{
    for (int i = 0; i < nx; ++i)
        for (int k = 0; k < nz; ++k)
        {
            float x = cx - half_w + (2.f * half_w / (nx - 1)) * i;
            float z = cz - half_h + (2.f * half_h / (nz - 1)) * k;
            push_point(buf, x, y, z, noise);
        }
}

// Vertical rectangular face (parallel to YZ plane, constant x)
inline void sample_face_yz(std::vector<uint8_t>& buf,
                             float x, float cy, float cz,
                             float half_d, float half_h,
                             int ny, int nz,
                             float noise = 0.003f)
{
    for (int j = 0; j < ny; ++j)
        for (int k = 0; k < nz; ++k)
        {
            float y = cy - half_d + (2.f * half_d / (ny - 1)) * j;
            float z = cz - half_h + (2.f * half_h / (nz - 1)) * k;
            push_point(buf, x, y, z, noise);
        }
}

// ── Object builders ────────────────────────────────────────────────────────

// TABLE
// 0.9 m along X, 1.2 m along Y, surface at z = -0.05
// Side faces visible to the sensor are sampled lightly.
inline void add_table(std::vector<uint8_t>& buf)
{
    const float cx   =  0.45f;   // centre along X (table spans 0 → 0.9)
    const float cy   =  0.00f;   // centred on EEF in Y
    const float z_top = -0.05f;  // table surface
    const float hw   =  0.45f;   // half-width in X
    const float hd   =  0.60f;   // half-depth in Y

    // Top surface
    sample_surface(buf, cx, cy, z_top, hw, hd, 30, 40, 0.002f);

    // Front edge face (visible at +X end, xz plane at y = cy+hd)
    sample_face_xz(buf, cx, cy + hd, z_top - 0.04f, hw, 0.04f, 20, 5, 0.003f);
    // Near edge face (xz plane at y = cy-hd)
    sample_face_xz(buf, cx, cy - hd, z_top - 0.04f, hw, 0.04f, 20, 5, 0.003f);
}

// WELLPLATE (standard 96-well, SBS footprint: 127.76 mm × 85.48 mm)
// Positioned at +X side of table, centred around x=0.65, y=0.15
// Top surface at z = -0.05 + 0.014 = -0.036 (plate is 14 mm tall)
inline void add_wellplate(std::vector<uint8_t>& buf)
{
    const float cx     =  0.65f;
    const float cy     =  0.15f;
    const float z_base = -0.05f;
    const float height =  0.014f;
    const float hw     =  0.06388f;   // 127.76/2 mm
    const float hd     =  0.04274f;   // 85.48/2 mm

    // Plate body top surface
    sample_surface(buf, cx, cy, z_base + height, hw, hd, 20, 14, 0.002f);

    // Side faces
    sample_face_xz(buf, cx,  cy + hd, z_base + height * 0.5f, hw, height * 0.5f, 12, 4, 0.002f);
    sample_face_xz(buf, cx,  cy - hd, z_base + height * 0.5f, hw, height * 0.5f, 12, 4, 0.002f);
    sample_face_yz(buf, cx + hw, cy,  z_base + height * 0.5f, hd, height * 0.5f,  8, 4, 0.002f);

    // 96 well openings (8 rows × 12 cols), 9 mm pitch
    // Represented as a small cluster of points dipping into each well
    const int rows = 8, cols = 12;
    const float pitch_x = 0.009f, pitch_y = 0.009f;
    const float ox = cx - (cols - 1) * pitch_x * 0.5f;
    const float oy = cy - (rows - 1) * pitch_y * 0.5f;
    const float well_z = z_base + height - 0.003f;  // slightly recessed

    std::uniform_real_distribution<float> jitter(-0.001f, 0.001f);
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
        {
            float wx = ox + c * pitch_x;
            float wy = oy + r * pitch_y;
            // 4 points per well rim
            push_point(buf, wx + 0.003f, wy,          well_z, 0.001f);
            push_point(buf, wx - 0.003f, wy,          well_z, 0.001f);
            push_point(buf, wx,          wy + 0.003f, well_z, 0.001f);
            push_point(buf, wx,          wy - 0.003f, well_z, 0.001f);
        }
}

// LIQUID HANDLER
// A tall gantry-style body at +Y, simplified as:
//   - Base plate on table surface
//   - Two vertical uprights
//   - Horizontal crossbar
//   - Pipette head block
// Centred around x=0.10, y=0.55
inline void add_liquid_handler(std::vector<uint8_t>& buf)
{
    const float cx   =  0.10f;
    const float cy   =  0.55f;
    const float z0   = -0.05f;   // table surface

    // Base plate
    sample_surface(buf, cx, cy, z0 + 0.005f, 0.12f, 0.10f, 12, 10, 0.002f);

    // Left upright  (yz face at x = cx - 0.10)
    sample_face_yz(buf, cx - 0.10f, cy, z0 + 0.20f, 0.04f, 0.20f, 4, 15, 0.003f);
    // Right upright (yz face at x = cx + 0.10)
    sample_face_yz(buf, cx + 0.10f, cy, z0 + 0.20f, 0.04f, 0.20f, 4, 15, 0.003f);

    // Horizontal crossbar top face
    sample_surface(buf, cx, cy, z0 + 0.40f, 0.10f, 0.03f, 14, 4, 0.002f);

    // Pipette head block (front face, xz plane)
    sample_face_xz(buf, cx, cy - 0.04f, z0 + 0.33f, 0.07f, 0.05f, 10, 6, 0.002f);
    // Pipette head bottom
    sample_surface(buf, cx, cy - 0.02f, z0 + 0.28f, 0.06f, 0.02f, 8, 3, 0.002f);

    // 8 pipette tips hanging down (single points per tip, more realistic depth)
    const float tip_z   = z0 + 0.27f;
    const float tip_cy  = cy - 0.03f;
    const float tip_ox  = cx - 0.035f;
    for (int t = 0; t < 8; ++t)
    {
        float tx = tip_ox + t * 0.01f;
        push_point(buf, tx, tip_cy, tip_z,        0.001f);
        push_point(buf, tx, tip_cy, tip_z - 0.01f, 0.001f);
        push_point(buf, tx, tip_cy, tip_z - 0.02f, 0.001f);
    }
}

// SHAKER MODULE
// Compact orbital shaker on -Y side
// Centred around x=0.20, y=-0.45
// Body box + platform surface + corner posts
inline void add_shaker(std::vector<uint8_t>& buf)
{
    const float cx   =  0.20f;
    const float cy   = -0.45f;
    const float z0   = -0.05f;
    const float hw   =  0.08f;   // half-width X
    const float hd   =  0.07f;   // half-depth Y
    const float body =  0.07f;   // body height

    // Body top surface (platform)
    sample_surface(buf, cx, cy, z0 + body, hw, hd, 14, 12, 0.002f);

    // Front face (toward -Y, visible to sensor)
    sample_face_xz(buf, cx, cy - hd, z0 + body * 0.5f, hw, body * 0.5f, 12, 6, 0.003f);

    // Right face (+X)
    sample_face_yz(buf, cx + hw, cy, z0 + body * 0.5f, hd, body * 0.5f, 10, 6, 0.003f);

    // Base plate on table
    sample_surface(buf, cx, cy, z0 + 0.004f, hw, hd, 10, 8, 0.002f);

    // 4 corner mounting posts (small vertical columns)
    float post_xs[] = {cx - hw + 0.01f, cx + hw - 0.01f,
                       cx - hw + 0.01f, cx + hw - 0.01f};
    float post_ys[] = {cy - hd + 0.01f, cy - hd + 0.01f,
                       cy + hd - 0.01f, cy + hd - 0.01f};
    for (int p = 0; p < 4; ++p)
        for (int k = 0; k < 8; ++k)
            push_point(buf, post_xs[p], post_ys[p],
                       z0 + (body / 7.f) * k, 0.002f);
}

// ── PointCloud2 message builder ────────────────────────────────────────────

inline sensor_msgs::msg::PointCloud2 build_cloud(rclcpp::Node::SharedPtr node)
{
    std::vector<uint8_t> data;
    data.reserve(200000);   // pre-alloc ~16 k points × 12 bytes

    add_table(data);
    add_wellplate(data);
    add_liquid_handler(data);
    add_shaker(data);

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = "world";
    msg.header.stamp    = node->now();
    msg.height          = 1;
    msg.fields.resize(3);

    auto mf = [](const std::string& name, uint32_t offset)
    {
        sensor_msgs::msg::PointField f;
        f.name     = name;
        f.offset   = offset;
        f.datatype = sensor_msgs::msg::PointField::FLOAT32;
        f.count    = 1;
        return f;
    };
    msg.fields[0] = mf("x", 0);
    msg.fields[1] = mf("y", 4);
    msg.fields[2] = mf("z", 8);

    msg.is_bigendian = false;
    msg.point_step   = 12;
    msg.is_dense     = true;
    msg.width        = static_cast<uint32_t>(data.size() / 12);
    msg.row_step     = msg.point_step * msg.width;
    msg.data         = std::move(data);
    return msg;
}

// ── Publisher helper (mirrors your original API) ───────────────────────────
// Call once and store the returned pair; do NOT let them go out of scope.
inline auto create_cloud_publisher(rclcpp::Node::SharedPtr node)
{
    auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/point_cloud", 10);

    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(200),
        [pub, node]() {          // captured by VALUE — no dangling refs
            pub->publish(build_cloud(node));
        });

    return std::make_pair(pub, timer);
}

}  // namespace dummy_cloud