// ==========================================================
// BEMT Solver Logic (Translated from Python)
// ==========================================================

// Helper function to replace np.linspace
function linspace(start, end, num) {
    const arr = [];
    const step = (end - start) / (num - 1);
    for (let i = 0; i < num; i++) {
        arr.push(start + (step * i));
    }
    return arr;
}

// Helper function to replace np.interp
function interp(x_new, x_arr, y_arr) {
    if (x_new <= x_arr[0]) return y_arr[0];
    if (x_new >= x_arr[x_arr.length - 1]) return y_arr[y_arr.length - 1];
    
    let i = x_arr.findIndex(x => x > x_new) - 1;
    if (i < 0) i = 0; // Should be covered by bounds check, but safe
    
    const x0 = x_arr[i];
    const x1 = x_arr[i + 1];
    const y0 = y_arr[i];
    const y1 = y_arr[i + 1];
    
    return y0 + (y1 - y0) * (x_new - x0) / (x1 - x0);
}

// Helper function to replace np.deg2rad
function deg2rad(deg) {
    return deg * (Math.PI / 180);
}

/**
 * Runs the BEMT algorithm for a single operating point.
 */
function bemSolver(V_inf, Omega, R, B, hub_radius, r_R_data, c_R_data, twist_data_deg, airfoil) {
    
    const N_elements = 40;
    const rho = 1.225;
    const tolerance = 1e-6;
    const max_iter = 100;
    
    // 1. Convert airfoil angles to radians
    const alpha_rad_data = airfoil.alpha_deg.map(deg2rad);
    const { cl: cl_data, cd: cd_data } = airfoil;
    
    // 2. Discretize blade
    const r_stations = linspace(r_R_data[0] * R, R, N_elements);
    const twist_rad_data = twist_data_deg.map(deg2rad);
    const r_data_abs = r_R_data.map(r => r * R);

    const twist_rad = r_stations.map(r => interp(r, r_data_abs, twist_rad_data));
    const chord = r_stations.map(r => interp(r, r_data_abs, c_R_data.map(c => c * R)));
    const dr = r_stations[1] - r_stations[0];

    let elem_Thrust = new Array(N_elements).fill(0);
    let elem_Torque = new Array(N_elements).fill(0);

    // 3. Loop over each blade element
    for (let i = 0; i < N_elements; i++) {
        const r = r_stations[i];
        let a = 0.1, ap = 0.01; // Initial guess

        for (let iter = 0; iter < max_iter; iter++) {
            const phi = Math.atan2(V_inf * (1 + a), Omega * r * (1 - ap));
            const alpha = twist_rad[i] - phi;
            
            const Cl = interp(alpha, alpha_rad_data, cl_data);
            const Cd = interp(alpha, alpha_rad_data, cd_data);
            
            // Note: Signs are for a turbine (power generating)
            const Cn = Cl * Math.cos(phi) + Cd * Math.sin(phi);
            const Ct = Cl * Math.sin(phi) - Cd * Math.cos(phi);

            // Prandtl's tip/hub loss factor
            const sinPhi = Math.sin(phi);
            let F = 1.0;
            if (Math.abs(sinPhi) > 1e-6) {
                const f_tip = (B / 2) * (R - r) / (r * Math.abs(sinPhi));
                const F_tip = (2 / Math.PI) * Math.acos(Math.exp(-f_tip));
                const f_hub = (B / 2) * (r - hub_radius) / (hub_radius * Math.abs(sinPhi));
                const F_hub = (2 / Math.PI) * Math.acos(Math.exp(-f_hub));
                F = F_tip * F_hub;
            }

            // Update induction factors
            const sigma = (B * chord[i]) / (2 * Math.PI * r);
            const a_new = 1 / ((4 * F * sinPhi**2) / (sigma * Cn) - 1);
            const ap_new = 1 / ((4 * F * sinPhi * Math.cos(phi)) / (sigma * Ct) + 1);
            
            // Relaxation
            a = 0.5 * a + 0.5 * a_new;
            ap = 0.5 * ap + 0.5 * ap_new;

            if (Math.abs(a - a_new) < tolerance && Math.abs(ap - ap_new) < tolerance) {
                break;
            }
        }
        
        const W_sq = (V_inf * (1 + a))**2 + (Omega * r * (1 - ap))**2;
        elem_Thrust[i] = 0.5 * rho * W_sq * B * chord[i] * Cn;
        elem_Torque[i] = 0.5 * rho * W_sq * B * chord[i] * Ct * r;
    }

    // 4. Integrate
    const Total_Thrust = elem_Thrust.reduce((sum, val) => sum + val, 0) * dr;
    const Total_Torque = elem_Torque.reduce((sum, val) => sum + val, 0) * dr;
    const Total_Power = Total_Torque * Omega;
    
    return { Total_Thrust, Total_Torque, Total_Power };
}
