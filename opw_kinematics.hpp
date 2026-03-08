#pragma once
/**
 * OPW Inverse Kinematics
 * ──────────────────────────────────────────────────────────────────────────
 * Implementation of:
 *   Brandstötter, Angerer, Hofbaur —
 *   "An Analytical Solution of the Inverse Kinematics Problem of Industrial
 *    Serial Manipulators with an Ortho-parallel Basis and a Spherical Wrist"
 *   Austrian Robotics Workshop, May 2014.
 *
 * Dependencies: Eigen (Matrix3d, Vector3d only)
 * ──────────────────────────────────────────────────────────────────────────
 */

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

// ═══════════════════════════════════════════════════════════════════════════
//  Veri Yapıları
// ═══════════════════════════════════════════════════════════════════════════

/**
 * OPW Parametreleri — makale Table I'e karşılık gelir.
 * Tüm uzunluklar metre, açılar radyan.
 *
 *  a1 : Omuz ofseti (x yönü)
 *  a2 : Dirsek ofseti
 *  b  : Yanal ofset (y yönü)
 *  c1 : Taban yüksekliği
 *  c2 : Omuz–dirsek kol uzunluğu
 *  c3 : Dirsek–bilek kol uzunluğu
 *  c4 : Bilek–uç işleyici uzunluğu
 */
struct OPWParameters {
    double a1, a2, b;
    double c1, c2, c3, c4;
    std::array<double,6> offsets = {0,0,0,0,0,0}; ///< eklem sıfır düzeltmeleri
    std::array<int,6>    signs   = {1,1,1,1,1,1};  ///< eklem yön işaretleri
};

/** Uç işleyici pozu: konum + dönme matrisi */
struct Pose {
    Eigen::Vector3d position;   ///< [x,y,z] metre
    Eigen::Matrix3d rotation;   ///< R0e (3×3)
};

/** Tek eklem açısı seti; valid=false → geometrik erişim yok */
struct JointSolution {
    std::array<double,6> q = {0,0,0,0,0,0};
    bool valid = false;
};

using IKSolutions = std::array<JointSolution, 8>;

// ═══════════════════════════════════════════════════════════════════════════
//  Yardımcı Fonksiyonlar
// ═══════════════════════════════════════════════════════════════════════════
namespace opw_internal {

constexpr double EPS = 1e-10;

inline double safe_acos(double v) {
    if (v >  1.0) v =  1.0;
    if (v < -1.0) v = -1.0;
    return std::acos(v);
}

inline bool is_valid(double v) { return std::isfinite(v); }

inline double normalize_angle(double a) {
    while (a >  M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
}

} // namespace opw_internal

// ═══════════════════════════════════════════════════════════════════════════
//  İleri Kinematik
// ═══════════════════════════════════════════════════════════════════════════
/**
 * İleri kinematik.
 *
 * Bölüm III.B.1 ve III.C denklemlerine dayanır.
 * FK denklemleri:
 *   cx1 = c2*sinθ2 + c3*sin(θ2+θ3) + a2*cos(θ2+θ3) + a1   [Denklem 2]
 *   cz1 = c2*cosθ2 + c3*cos(θ2+θ3) - a2*sin(θ2+θ3)         [Denklem 4]
 *   C_x0 = cx1*cosθ1 - b*sinθ1                               [Denklem 5]
 *   C_y0 = cx1*sinθ1 + b*cosθ1                               [Denklem 6]
 *   C_z0 = cz1 + c1                                          [Denklem 7]
 */
inline Pose forward_kinematics(const OPWParameters& p,
                               const std::array<double,6>& q)
{
    double t[6];
    for (int i = 0; i < 6; i++)
        t[i] = q[i] * p.signs[i] + p.offsets[i];

    const double s1  = std::sin(t[0]), c1  = std::cos(t[0]);
    const double s2  = std::sin(t[1]), c2_ = std::cos(t[1]);
    const double s4  = std::sin(t[3]), c4_ = std::cos(t[3]);
    const double s5  = std::sin(t[4]), c5  = std::cos(t[4]);
    const double s6  = std::sin(t[5]), c6  = std::cos(t[5]);
    const double s23 = std::sin(t[1]+t[2]), c23 = std::cos(t[1]+t[2]);

    // ── C noktası koordinatları (Denklem 2–7)
    const double cx1 = p.c2*s2 + p.c3*s23 + p.a2*c23 + p.a1;
    const double cz1 = p.c2*c2_ + p.c3*c23 - p.a2*s23;

    const double Cx = cx1*c1 - p.b*s1;
    const double Cy = cx1*s1 + p.b*c1;
    const double Cz = cz1 + p.c1;

    // ── R0c: taban → C koordinat çerçevesi dönme matrisi
    Eigen::Matrix3d R0c;
    R0c(0,0)= c1*c23; R0c(0,1)=-s1; R0c(0,2)= c1*s23;
    R0c(1,0)= s1*c23; R0c(1,1)= c1; R0c(1,2)= s1*s23;
    R0c(2,0)=   -s23; R0c(2,1)=  0; R0c(2,2)=    c23;

    // ── Rce: küresel bilek dönme matrisi (ZYZ Euler — Bölüm III.C)
    Eigen::Matrix3d Rce;
    Rce(0,0)= c4_*c5*c6-s4*s6; Rce(0,1)=-c4_*c5*s6-s4*c6; Rce(0,2)= c4_*s5;
    Rce(1,0)= s4*c5*c6+c4_*s6; Rce(1,1)=-s4*c5*s6+c4_*c6; Rce(1,2)= s4*s5;
    Rce(2,0)=          -s5*c6; Rce(2,1)=           s5*s6;  Rce(2,2)=    c5;

    // ── R0e = R0c · Rce
    Eigen::Matrix3d R0e = R0c * Rce;

    // ── u = C + c4 · R0e · [0,0,1]ᵀ
    Eigen::Vector3d pos;
    pos(0) = Cx + p.c4 * R0e(0,2);
    pos(1) = Cy + p.c4 * R0e(1,2);
    pos(2) = Cz + p.c4 * R0e(2,2);

    return {pos, R0e};
}

// ═══════════════════════════════════════════════════════════════════════════
//  Ters Kinematik
// ═══════════════════════════════════════════════════════════════════════════
/**
 * OPW Ters Kinematik — en fazla 8 analitik çözüm.
 *
 * Algoritma:
 *  1) C = u - c4·R0e[:,2]                         (bilek merkezini bul)
 *  2) |cx1| = sqrt(cx0²+cy0²-b²)                  (yatay mesafe)
 *  3) İki postür çifti için (cx1>0 ön, cx1<0 arka):
 *     a) nx1 = cx1 - a1                (işaretli G2→C x-bileşeni, Denklem 8)
 *     b) s1² = nx1² + dz²              (Denklem 9)
 *     c) θ1 = atan2(cy0,cx0) - atan2(b,cx1)
 *     d) ψ2 = acos((s1²+c2²-k²)/(2·s1·c2))  (Denklem 15)
 *     e) θ2 = ∓ψ2 + atan2(nx1,dz)     (Denklem 14/16)
 *     f) θ3 = ±acos((s1²-c2²-k²)/(2·c2·k)) - ψ3  (Denklem 10)
 *        NOT: θ2 ve θ3 ters işaret çiftleri paylaşır (−ψ2,+acos) ve (+ψ2,−acos)
 *  4) Her konum çözümü için R0c hesapla, Rce = R0cᵀ·R0e  (Denklem 17)
 *  5) θ5 = atan2(sin,cos) from Rce(2,2)
 *     θ4 = atan2(Rce(1,2), Rce(0,2))
 *     θ6 = atan2(Rce(2,1),-Rce(2,0))
 *     → 2 bilek çözümü: (θ4, θ5, θ6) ve (θ4+π, -θ5, θ6-π)
 *  Toplam: 4 konum × 2 bilek = 8 çözüm
 */
inline IKSolutions inverse_kinematics(const OPWParameters& params,
                                      const Pose& pose)
{
    using namespace opw_internal;

    IKSolutions solutions;
    const Eigen::Matrix3d& R0e = pose.rotation;
    const Eigen::Vector3d& u   = pose.position;

    // ─── ADIM 1: C noktası ───────────────────────────────────────────────
    const double cx0 = u(0) - params.c4 * R0e(0,2);
    const double cy0 = u(1) - params.c4 * R0e(1,2);
    const double cz0 = u(2) - params.c4 * R0e(2,2);

    // ─── ADIM 2: Temel skalerler ─────────────────────────────────────────
    const double xy_sq  = cx0*cx0 + cy0*cy0 - params.b*params.b;
    if (xy_sq < 0.0) return solutions;  // erişilemez
    const double cx1_abs = std::sqrt(xy_sq);  // = |cx1|

    const double dz  = cz0 - params.c1;
    const double k2  = params.a2*params.a2 + params.c3*params.c3;
    const double k   = std::sqrt(k2);
    const double c2  = params.c2;
    const double c2sq = c2*c2;
    const double psi3 = std::atan2(params.a2, params.c3);

    // ─── ADIM 3–5: 4 konum çözümü ────────────────────────────────────────
    // İki postür çifti: cx1 = +|cx1| (ön) ve cx1 = −|cx1| (arka)
    // Her çiftten iki dirsek konfigürasyonu (aşağı/yukarı) → toplam 4
    struct PosSol { double t1, t2, t3; bool ok; };
    PosSol pos_sol[4];
    int n_pos = 0;

    for (int scx : {+1, -1}) {
        const double cx1 = scx * cx1_abs;
        // nx1 = cx1 - a1: G2→C vektörünün x1-bileşeni (işaretli, Denklem 8)
        const double nx1 = cx1 - params.a1;
        const double s1sq = nx1*nx1 + dz*dz;
        const double s1v  = std::sqrt(s1sq);
        if (s1v < EPS) continue;

        // Üçgen eşitsizliği kontrolü: |c2-k| <= s1 <= c2+k
        // Bu sağlanmazsa kol bu konfigürasyona erişemez → çözüm yok
        const double s1_lo = std::abs(c2 - k);
        const double s1_hi = c2 + k;
        if (s1v < s1_lo - EPS || s1v > s1_hi + EPS) continue;

        // θ1: atan2(cy0,cx0) = θ1 + atan2(b,cx1)
        const double t1 = std::atan2(cy0, cx0) - std::atan2(params.b, cx1);

        // ψ2 (kosinüs teoremi, Denklem 15)
        const double cos_psi2 = (s1sq + c2sq - k2) / (2.0*s1v*c2 + EPS);
        const double psi2     = safe_acos(cos_psi2);
        const double base     = std::atan2(nx1, dz);  // işaretli nx1 kritik!

        // θ3 için acos (Denklem 10)
        const double cos_t3  = (s1sq - c2sq - k2) / (2.0*c2*k + EPS);
        const double acos_t3 = safe_acos(cos_t3);

        // İşaret çiftleri: (θ2 dirsek aşağı, θ3 pozitif) & (θ2 dirsek yukarı, θ3 negatif)
        for (int s = 0; s < 2; s++) {
            const double s2 = (s==0) ? -1.0 : +1.0;  // θ2 işareti
            const double s3 = -s2;                     // θ3 işareti (ters!)
            const double t2 = s2 * psi2 + base;
            const double t3 = s3 * acos_t3 - psi3;
            const bool ok = is_valid(t2) && is_valid(t3);
            if (n_pos < 4) pos_sol[n_pos++] = {t1, t2, t3, ok};
        }
    }

    // ─── ADIM 6–9: Yönelim — her konum çözümü için 2 bilek ─────────────
    int sol_idx = 0;
    for (int p = 0; p < n_pos && sol_idx < 8; p++) {
        if (!pos_sol[p].ok) { sol_idx += 2; continue; }

        const double t1  = pos_sol[p].t1;
        const double t2  = pos_sol[p].t2;
        const double t3  = pos_sol[p].t3;

        const double s1p  = std::sin(t1), c1p  = std::cos(t1);
        const double s23p = std::sin(t2+t3), c23p = std::cos(t2+t3);

        // R0c (Bölüm III.B.1)
        Eigen::Matrix3d R0c;
        R0c(0,0)= c1p*c23p; R0c(0,1)=-s1p; R0c(0,2)= c1p*s23p;
        R0c(1,0)= s1p*c23p; R0c(1,1)= c1p; R0c(1,2)= s1p*s23p;
        R0c(2,0)=    -s23p; R0c(2,1)=   0; R0c(2,2)=      c23p;

        // Rce = R0cᵀ · R0e  (Denklem 17)
        Eigen::Matrix3d Rce = R0c.transpose() * R0e;

        // θ5 from Rce(2,2) = cos(θ5)
        const double mp   = Rce(2,2);
        const double t5p  = std::atan2(std::sqrt(std::max(0.0, 1.0-mp*mp)), mp);

        for (int w = 0; w < 2 && sol_idx < 8; w++, sol_idx++) {
            double t4, t5, t6;
            if (w == 0) {
                t5 = t5p;
                if (std::abs(std::sin(t5)) < EPS) {
                    // bilek singülaritesi: θ4+θ6 belirsiz, θ4=0 seç
                    t4 = 0.0;
                    t6 = std::atan2(-Rce(0,1), Rce(0,0));
                } else {
                    t4 = std::atan2(Rce(1,2), Rce(0,2));
                    t6 = std::atan2(Rce(2,1), -Rce(2,0));
                }
            } else {
                // İkinci bilek çözümü: θ5 negatif → θ4+π, θ6-π
                t5 = -t5p;
                if (std::abs(std::sin(t5)) < EPS) {
                    // Singülaritede θ4+π, 0, θ6-π de geçerli bir çözümdür
                    t4 = M_PI;
                    t6 = std::atan2(-Rce(0,1), Rce(0,0)) - M_PI;
                } else {
                    t4 = std::atan2(Rce(1,2), Rce(0,2)) + M_PI;
                    t6 = std::atan2(Rce(2,1), -Rce(2,0)) - M_PI;
                }
            }

            // Ofset ve işaret geri dönüşümü
            JointSolution& sol = solutions[sol_idx];
            double raw[6] = {t1,t2,t3,t4,t5,t6};
            for (int i = 0; i < 6; i++) {
                sol.q[i] = normalize_angle(
                    (raw[i] - params.offsets[i]) * params.signs[i]);
            }
            sol.valid = true;
        }
    }

    return solutions;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Hazır Robot Parametreleri — Makale Table I
// ═══════════════════════════════════════════════════════════════════════════
namespace opw_robots {

inline OPWParameters kuka_kr6_r700()  { return {0.025,-0.035,0.0, 0.400,0.315,0.365,0.080}; }
inline OPWParameters abb_irb2400()    { return {0.100,-0.135,0.0, 0.615,0.705,0.755,0.085}; }
inline OPWParameters fanuc_r2000ib()  { return {0.720,-0.225,0.0, 0.600,1.075,1.280,0.235}; }
inline OPWParameters puma560()        { return {0.0,-0.02032,0.14909,0.6604,0.4318,0.43307,0.05625}; }
inline OPWParameters staubli_tx40()   { return {0.0,0.0,0.035, 0.320,0.225,0.225,0.065}; }

} // namespace opw_robots
