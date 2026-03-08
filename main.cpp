#include <iostream>
#include <iomanip>
#include "opw_kinematics.hpp"

double deg(double r){ return r*180.0/M_PI; }
double rad(double d){ return d*M_PI/180.0; }

void printPose(const Pose& T, const std::string& label){
    std::cout << "\n  " << label << "\n";
    std::cout << "  Rotasyon:\n" << T.rotation << "\n";
    std::cout << "  Pozisyon: ["
              << T.position(0) << ", "
              << T.position(1) << ", "
              << T.position(2) << "]\n";
}

void checkRoundtrip(const OPWParameters& params,
                    const std::array<double,6>& q_in,
                    const std::string& name)
{
    std::cout << "\n════════════════════════════════════════════\n";
    std::cout << "  " << name << "\n";
    std::cout << "════════════════════════════════════════════\n";

    // Girdi açıları
    std::cout << "  Girdi q [°]: [";
    for(int i=0;i<6;i++)
        std::cout<<std::fixed<<std::setprecision(1)<<deg(q_in[i])<<(i<5?", ":"");
    std::cout << "]\n";

    // İleri kinematik
    Pose T = forward_kinematics(params, q_in);
    printPose(T, "Hedef Pose (FK)");

    // Ters kinematik
    IKSolutions sols = inverse_kinematics(params, T);

    std::cout << "\n  IK Çözümleri (° cinsinden):\n";
    std::cout << "  " << std::string(75, '-') << "\n";

    int valid = 0;
    double best_pos_err = 1e9;

    for(int i=0;i<8;i++){
        const auto& s = sols[i];
        std::cout << "  [" << i+1 << "] ";
        if(!s.valid){
            std::cout << "ERİŞİLEMEZ\n";
            continue;
        }
        ++valid;

        for(int j=0;j<6;j++)
            std::cout<<std::setw(8)<<std::fixed<<std::setprecision(2)
                     <<deg(s.q[j])<<(j<5?",":"");

        // Gidiş-dönüş hatası
        Pose T2 = forward_kinematics(params, s.q);
        double pe = std::sqrt(
            std::pow(T2.position(0)-T.position(0),2)+
            std::pow(T2.position(1)-T.position(1),2)+
            std::pow(T2.position(2)-T.position(2),2));

        best_pos_err = std::min(best_pos_err, pe);

        std::cout << "  | err=" << std::scientific
                  << std::setprecision(1) << pe << " m";
        if(pe > 1e-6) std::cout << " ⚠";
        std::cout << "\n";
    }

    std::cout << "  " << std::string(75, '-') << "\n";
    std::cout << "  Geçerli: " << valid << "/8"
              << "  |  En iyi pos hatası: "
              << std::scientific << best_pos_err << " m\n";
}

int main(){
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════╗\n";
    std::cout << "║     OPW Inverse Kinematics — Brandstötter et al. 2014   ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════╝\n";

    // ── Test 1: KUKA KR6 R700 sixx ───────────────────────────
    {
        auto p = opw_robots::kuka_kr6_r700();
        checkRoundtrip(p,
            {rad(30),rad(-45),rad(60),rad(15),rad(-30),rad(90)},
            "KUKA KR6 R700 sixx");
    }

    // ── Test 2: ABB IRB 2400/10 (home) ───────────────────────
    {
        auto p = opw_robots::abb_irb2400();
        checkRoundtrip(p,
            {rad(0),rad(0),rad(0),rad(0),rad(0),rad(0)},
            "ABB IRB 2400/10 — Home pozisyonu");
    }

    // ── Test 3: Stäubli TX40 (b ≠ 0) ─────────────────────────
    {
        auto p = opw_robots::staubli_tx40();
        checkRoundtrip(p,
            {rad(-20),rad(30),rad(10),rad(45),rad(-60),rad(15)},
            "Stäubli TX40 (yanal ofset b=35mm)");
    }

    // ── Test 4: Fanuc R-2000iB (büyük robot) ─────────────────
    {
        auto p = opw_robots::fanuc_r2000ib();
        checkRoundtrip(p,
            {rad(45),rad(-30),rad(15),rad(-90),rad(60),rad(0)},
            "Fanuc R-2000iB/200R");
    }

    // ── Test 5: PUMA 560 (a2<0, b≠0) ─────────────────────────
    {
        auto p = opw_robots::puma560();
        checkRoundtrip(p,
            {rad(0),rad(45),rad(-90),rad(0),rad(45),rad(0)},
            "Unimation PUMA 560");
    }

    // ── Test 6: Bilek singülaritesi (θ5≈0) ───────────────────
    {
        auto p = opw_robots::kuka_kr6_r700();
        checkRoundtrip(p,
            {rad(45),rad(-30),rad(20),rad(10),rad(0.001),rad(5)},
            "KUKA KR6 — Bilek singülaritesi (θ5≈0)");
    }

    return 0;
}
