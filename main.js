// ==========================================================
// Main script to connect UI to the BEMT solver
// ==========================================================

// Global chart objects so we can destroy them before redrawing
let cpChart = null;
let ctChart = null;

// --- 1. Initialization ---
document.addEventListener('DOMContentLoaded', () => {
    // Populate airfoil dropdown
    const airfoilSelect = document.getElementById('airfoil-select');
    for (const name in AIRFOIL_DB) {
        const option = document.createElement('option');
        option.value = name;
        option.textContent = name;
        airfoilSelect.appendChild(option);
    }
    
    // Add event listener to the button
    document.getElementById('run-button').addEventListener('click', runSimulation);
});

// --- 2. Main Simulation Function ---
function runSimulation() {
    const button = document.getElementById('run-button');
    const spinner = document.getElementById('loading-spinner');
    
    button.disabled = true;
    spinner.style.display = 'block';

    // Use setTimeout to allow the UI to update (show spinner) before freezing
    setTimeout(() => {
        try {
            // --- A. Get all inputs from the DOM ---
            const airfoilName = document.getElementById('airfoil-select').value;
            const airfoil = AIRFOIL_DB[airfoilName];
            
            const B = parseFloat(document.getElementById('num-blades').value);
            const R = parseFloat(document.getElementById('radius').value);
            const hub_radius = parseFloat(document.getElementById('hub-radius').value);
            const RPM = parseFloat(document.getElementById('rpm').value);
            const Omega = RPM * (2 * Math.PI / 60);

            // Parse geometry
            const geomText = document.getElementById('geom-data').value;
            const geomLines = geomText.trim().split('\n');
            const r_R_data = [];
            const c_R_data = [];
            const twist_data_deg = [];
            
            geomLines.forEach(line => {
                const [r_R, c_R, twist] = line.split(',').map(parseFloat);
                if (!isNaN(r_R) && !isNaN(c_R) && !isNaN(twist)) {
                    r_R_data.push(r_R);
                    c_R_data.push(c_R);
                    twist_data_deg.push(twist);
                }
            });

            // Get sweep settings
            const v_start = parseFloat(document.getElementById('v-start').value);
            const v_end = parseFloat(document.getElementById('v-end').value);
            const v_steps = parseInt(document.getElementById('v-steps').value);
            const v_inf_range = linspace(v_start, v_end, v_steps);

            // --- B. Run the sweep ---
            const tsr_results = [];
            const cp_results = [];
            const ct_results = [];
            const Area = Math.PI * R**2;
            const rho = 1.225;

            for (const v_inf of v_inf_range) {
                if (v_inf < 0.1) continue;

                const { Total_Thrust, Total_Power } = bemSolver(
                    v_inf, Omega, R, B, hub_radius,
                    r_R_data, c_R_data, twist_data_deg,
                    airfoil
                );

                const P_avail = 0.5 * rho * Area * v_inf**3;
                const Cp = Total_Power / P_avail;
                const Ct = Total_Thrust / (0.5 * rho * Area * v_inf**2);
                const tsr = (Omega * R) / v_inf;

                tsr_results.push(tsr);
                cp_results.push(Cp);
                ct_results.push(Ct);
            }

            // --- C. Process and Display Results ---
            displayResults(tsr_results, cp_results, ct_results);

        } catch (error) {
            alert("Error during simulation: " + error.message);
            console.error(error);
        } finally {
            // Re-enable button and hide spinner
            button.disabled = false;
            spinner.style.display = 'none';
        }
    }, 10); // 10ms delay
}

// --- 3. Display and Plotting ---
function displayResults(tsr, cp, ct) {
    // Find max Cp
    let max_cp = 0;
    let tsr_at_max_cp = 0;
    for (let i = 0; i < cp.length; i++) {
        if (cp[i] > max_cp) {
            max_cp = cp[i];
            tsr_at_max_cp = tsr[i];
        }
    }
    
    // Update summary
    const summary = document.getElementById('results-summary');
    summary.innerHTML = `
        <p>Simulation Complete. 
        <strong>Max C<sub>P</sub>: ${max_cp.toFixed(3)}</strong> 
        at <strong>TSR: ${tsr_at_max_cp.toFixed(2)}</strong></p>
    `;

    // --- Plot Cp Chart ---
    const cpCtx = document.getElementById('cp-chart').getContext('2d');
    if (cpChart) cpChart.destroy(); // Destroy old chart
    cpChart = new Chart(cpCtx, {
        type: 'line',
        data: {
            labels: tsr.map(v => v.toFixed(2)),
            datasets: [
                {
                    label: 'Cp (Power Coefficient)',
                    data: cp,
                    borderColor: 'rgba(0, 123, 255, 1)',
                    backgroundColor: 'rgba(0, 123, 255, 0.1)',
                    borderWidth: 2,
                    pointRadius: 3,
                    fill: true
                },
                { // Betz Limit
                    label: 'Betz Limit (0.593)',
                    data: tsr.map(() => 16/27),
                    borderColor: 'rgba(255, 99, 132, 1)',
                    borderWidth: 2,
                    pointRadius: 0,
                    borderDash: [5, 5],
                    fill: false
                }
            ]
        },
        options: {
            responsive: true,
            scales: {
                x: { title: { display: true, text: 'Tip-Speed Ratio (TSR)' } },
                y: { title: { display: true, text: 'Cp' }, min: 0, max: 0.7 }
            }
        }
    });

    // --- Plot Ct Chart ---
    const ctCtx = document.getElementById('ct-chart').getContext('2d');
    if (ctChart) ctChart.destroy(); // Destroy old chart
    ctChart = new Chart(ctCtx, {
        type: 'line',
        data: {
            labels: tsr.map(v => v.toFixed(2)),
            datasets: [{
                label: 'Ct (Thrust Coefficient)',
                data: ct,
                borderColor: 'rgba(40, 167, 69, 1)',
                backgroundColor: 'rgba(40, 167, 69, 0.1)',
                borderWidth: 2,
                pointRadius: 3,
                fill: true
            }]
        },
        options: {
            responsive: true,
            scales: {
                x: { title: { display: true, text: 'Tip-Speed Ratio (TSR)' } },
                y: { title: { display: true, text: 'Ct' }, min: 0 }
            }
        }
    });
}
