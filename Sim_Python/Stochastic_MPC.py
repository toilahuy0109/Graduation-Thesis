
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from math import erf, sqrt

# =========================================================
# UPH-PCE + Formation Tracking Demo
# =========================================================

dt = 0.1
Tsim = 50.0
Nsim = int(Tsim/dt)

vp = 1.2
L = 0.4

sigma_deg = 8.0
sigma = np.deg2rad(sigma_deg)

delta_min = np.deg2rad(-15)
delta_max = np.deg2rad(15)

Tu = 2.0
Tp = 5.0

Nu = int(Tu/dt)
Np = int(Tp/dt)

Nc = 25

Ndrone = 10
Rform = 5.0
Robs = 10.0

wc = 2.0
wt = 0.1

# ---------------------------------------------------------
# Truncated Gaussian sampling
# ---------------------------------------------------------

def sample_truncated():
    while True:
        x = np.random.normal(0, sigma)
        if delta_min <= x <= delta_max:
            return x

# ---------------------------------------------------------
# Dubins propagation
# ---------------------------------------------------------

def dubins_step(x,y,th,delta):
    th2 = th + vp/L*np.tan(delta)*dt
    x2 = x + vp*np.cos(th)*dt
    y2 = y + vp*np.sin(th)*dt
    return x2,y2,th2

# ---------------------------------------------------------
# PCE basis
# ---------------------------------------------------------

def basis(x):
    return np.array([1.0, x, x*x-1.0])

# ---------------------------------------------------------
# UPH-PCE prediction
# ---------------------------------------------------------

def uph_pce_predict(state):

    samples = np.array([sample_truncated() for _ in range(Nc)])

    mu_hist = []
    sig_hist = []

    for k in range(Nu):

        X = np.zeros((Nc,2))

        for i,w in enumerate(samples):

            x,y,th = state

            for _ in range(k+1):
                x,y,th = dubins_step(x,y,th,w)

            X[i,:] = [x,y]

        Phi = np.vstack([basis(w/sigma) for w in samples])

        cx = np.linalg.lstsq(Phi,X[:,0],rcond=None)[0]
        cy = np.linalg.lstsq(Phi,X[:,1],rcond=None)[0]

        mux = cx[0]
        muy = cy[0]

        varx = cx[1]**2 + cx[2]**2
        vary = cy[1]**2 + cy[2]**2

        mu_hist.append([mux,muy])
        sig_hist.append(np.diag([max(varx,1e-3),max(vary,1e-3)]))

    return np.array(mu_hist), sig_hist

# ---------------------------------------------------------
# Arc formation
# ---------------------------------------------------------

angles = np.linspace(-np.pi/2,np.pi/2,Ndrone)

local_pts = np.vstack([
    Rform*np.cos(angles),
    Rform*np.sin(angles)
]).T

def drones(center,theta):
    R = np.array([
        [np.cos(theta),-np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])
    return (R @ local_pts.T).T + center

# ---------------------------------------------------------
# Coverage potential
# ---------------------------------------------------------

def gamma_value(center,theta,mu,Sigma):

    pts = drones(center,theta)

    S = Sigma + (Robs**2)*np.eye(2)
    invS = np.linalg.inv(S)

    g = 0.0

    for p in pts:
        d = mu-p
        g += np.exp(-0.5*d.T@invS@d)

    return g

# ---------------------------------------------------------
# Simulation
# ---------------------------------------------------------

xp = np.zeros(Nsim)
yp = np.zeros(Nsim)
thp = np.zeros(Nsim)

xc = np.zeros(Nsim)
yc = np.zeros(Nsim)
thetac = np.zeros(Nsim)

xc[0] = -10

coverage_log = []
dist_log = []

for k in range(Nsim-1):

    delta = sample_truncated()

    xp[k+1],yp[k+1],thp[k+1] = dubins_step(
        xp[k],yp[k],thp[k],delta
    )

    state = np.array([xp[k+1],yp[k+1],thp[k+1]])

    mu_hist,sig_hist = uph_pce_predict(state)

    mu = mu_hist[-1]
    Sigma = sig_hist[-1]

    z0 = np.array([xc[k],yc[k],thetac[k]])

    def objective(z):

        center = z[:2]
        theta = z[2]

        g = gamma_value(
            center,
            theta,
            mu,
            Sigma
        )

        track = np.sum((center-mu)**2)

        return -wc*g + wt*track

    res = minimize(
        objective,
        z0,
        method="BFGS"
    )

    z = res.x

    alpha = 0.15

    xc[k+1] = xc[k] + alpha*(z[0]-xc[k])
    yc[k+1] = yc[k] + alpha*(z[1]-yc[k])
    thetac[k+1] = thetac[k] + alpha*(z[2]-thetac[k])

    coverage_log.append(
        gamma_value(
            np.array([xc[k+1],yc[k+1]]),
            thetac[k+1],
            mu,
            Sigma
        )
    )

    dist_log.append(
        np.linalg.norm(
            np.array([xc[k+1],yc[k+1]]) - np.array([xp[k+1],yp[k+1]])
        )
    )

# ---------------------------------------------------------
# Plots
# ---------------------------------------------------------

plt.figure(figsize=(8,8))
plt.plot(xp,yp,label="Prey")
plt.plot(xc,yc,label="Formation Center")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.title("Trajectory")

plt.figure()
plt.plot(coverage_log)
plt.grid(True)
plt.title("Coverage Potential")

plt.figure()
plt.plot(dist_log)
plt.grid(True)
plt.title("Distance to Prey")

plt.show()
