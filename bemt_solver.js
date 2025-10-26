// ==========================================================
// BEMT Solver Logic (Translated from Python)
// ==========================================================

// Helper function to replace np.linspace
function linspace(start, end, num) {
    var arr = [];
    var step = (end - start) / (num - 1);
    for (var i = 0; i < num; i++) {
        arr.push(start + (step * i));
    }
    return arr;
}

// Helper function to replace np.interp
function interp(x_new, x_arr, y_arr) {
    if (x_new <= x_arr[0]) return y_arr[0];
    if (x_new >= x_arr[x_arr.length - 1]) return y_arr[y_arr.length - 1];
    
    var i = x_arr.findIndex(x => x > x_new) - 1;
    if (i < 0) i = 0; 
    
    var x0 = x_arr[i];
    var x1 = x_arr[i + 1];
    var y0 = y_arr[i];
    var y1 = y_arr[i + 1];
    
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
    
    var N_elements = 40;
    var rho = 1.225;
    var tolerance = 1e-6;
    var max_iter = 100;
    
    // 1. Convert airfoil angles to radians
    var alpha_rad_data = airfoil.alpha_deg.map(deg2rad);
    var cl_data = airfoil.cl;
    var cd_data = airfoil.cd;
    
    // 2. Discretize blade
    var r_stations = linspace(r_R_data[0] * R, R, N_elements);
    var twist_rad_data = twist_data_deg.map(deg2rad);
    var r_data_abs = r_R_data.map(r => r * R);

    var twist_rad = r_stations.map(r => interp(r, r_data_abs, twist_rad_data));
    var chord = r_stations.map(r => interp(r, r_data_abs, c_R_data.map(c => c * R)));
    var dr = r_stations[1] - r_stations[0];

    var elem_Thrust = new Array(N_elements).fill(0);
    var elem_Torque = new Array(N_elements).fill(0);

    // 3. Loop over each blade element
    for (var i = 0; i < N_elements; i++) {
        var r = r_stations[i];
        var a = 0.1, ap = 0.01; // Initial guess

        for (var iter = 0; iter < max_iter; iter++) {
            var phi = Math.atan2(V_inf * (1 + a), Omega * r * (1 - ap));
            var alpha = twist_rad[i] - phi;
            
            var Cl = interp(alpha, alpha_rad_data, cl_data);
            var Cd = interp(alpha, alpha_rad_data, cd_data);
            
            // Note: Signs are for a turbine (power generating)
            // ==========================================================
            //  !!! THIS IS THE FIX !!!
            //  The variables are now 'cn' and 'ct' (lowercase)
            // ==========================================================
            var cn = Cl * Math.cos(phi) + Cd * Math.sin(phi);
            var ct = Cl * Math.sin(phi) - Cd * Math.cos(phi);

            // Prandtl's tip/hub loss factor
            var sinPhi = Math.sin(phi);
            var F = 1.0;
            if (Math.abs(sinPhi) > 1e-6) {
                var f_tip = (B / 2) * (R - r) / (r * Math.abs(sinPhi));
                var F_tip = (2 / Math.PI) * Math.acos(Math.exp(-f_tip));
                var f_hub = (B / 2) * (r - hub_radius) / (hub_radius * Math.abs(sinPhi));
                var F_hub = (2 / Math.PI) * Math.acos(Math.exp(-f_hub));
                F = F_tip * F_hub;
            }

            // Update induction factors
            var sigma = (B * chord[i]) / (2 * Math.PI * r);

            // ==========================================================
            //  !!! THIS IS THE FIX !!!
            //  Using 'cn' and 'ct' (lowercase) to match the definition
            //  Also using Math.pow() for better browser compatibility
            // ==========================================================
            var a_new = 1 / ((4 * F * Math.pow(sinPhi, 2)) / (sigma * cn) - 1);
            var ap_new = 1 / ((4 * F * sinPhi * Math.cos(phi)) / (sigma * ct) + 1);
            
            // Relaxation
            a = 0.5 * a + 0.5 * a_new;
            ap = 0.5 * ap + 0.5 * ap_new;

            if (Math.abs(a - a_new) < tolerance && Math.abs(ap - ap_new) < tolerance) {
                break;
            }
        }
        
        var W_sq = Math.pow(V_inf * (1 + a), 2) + Math.pow(Omega * r * (1 - ap), 2);
        elem_Thrust[i] = 0.5 * rho * W_sq * B * chord[i] * cn;
        elem_Torque[i] = 0.5 * rho * W_sq * B * chord[i] * ct * r;
    }

    // 4. Integrate
    var Total_Thrust = elem_Thrust.reduce((sum, val) => sum + val, 0) * dr;
    var Total_Torque = elem_Torque.reduce((sum, val) => sum + val, 0) * dr;
    var Total_Power = Total_Torque * Omega;
    
    return { Total_Thrust, Total_Torque, Total_Power };
}
