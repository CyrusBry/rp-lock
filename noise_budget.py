import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams.update({"font.family":"serif","font.size":9,"axes.labelsize":9.5,"axes.titlesize":10.5,
                     "legend.fontsize":8,"mathtext.fontset":"cm"})
tau=9.8e-6; K=30e6  # [s], [Hz/mA]
i=np.logspace(-2,np.log10(30),400)          # [uA RMS]
dnu=K*i*1e-3                                  # [Hz RMS]
phi=2*np.pi*tau*dnu                           # [rad RMS]
fig,ax=plt.subplots(figsize=(7.2,4.8))
ax.loglog(i,phi,color="#1f4e79",lw=2,label=r"$\sigma_\varphi = 2\pi\,\tau_d\,K_I\, i_n$  ($\tau_d = 9.8\,\mu$s, $K_I = 30$ MHz/mA)")
ax.axhspan(1,100,color="#d9534f",alpha=0.10,lw=0)
ax.axhspan(0.5,1,color="#f0ad4e",alpha=0.12,lw=0)
ax.axhspan(1e-3,0.5,color="#5cb85c",alpha=0.10,lw=0)
ax.axhline(1,color="#d9534f",lw=1,ls="--"); ax.axhline(0.5,color="#f0ad4e",lw=1,ls="--")
ax.text(0.0115,2.6,"No Coherent Line (Carrier < 37 %): Loop Cannot Hold A Fringe",color="#a94442",fontsize=8)
ax.text(2.6,0.70,"Holds, Slope 60--88 %",color="#8a6d3b",fontsize=8,bbox=dict(boxstyle="round,pad=0.15",fc="white",ec="none",alpha=0.85))
ax.text(1.6,0.16,"Lock Holds\n(Slope > 88 %, Carrier > 78 %)",color="#3c763d",fontsize=8,va="center")
pts=[(10,"CLD1015, Filter Off\n(10 $\\mu$A, 250 kHz Mod BW)","#d9534f","o"),
     (5,"CLD1015, Filter On\n(5 $\\mu$A, 7.5 kHz Mod BW)","#d9534f","s"),
     (0.65,"CTL200-600 (650 nA)","#1f4e79","^"),
     (0.22,"CTL200-200 (220 nA)\nBuild 2B","#3c763d","^"),
     (0.13,"CTL200-100 (130 nA)","#3c763d","^")]
for x,lab,c,m in pts:
    y=2*np.pi*tau*K*x*1e-3
    ax.plot(x,y,marker=m,color=c,ms=7,ls="none",zorder=5)
    place={10:(0.68,1.0,"right"),5:(0.68,1.0,"right"),0.65:(1.18,0.50,"left"),0.22:(1.35,0.40,"left"),0.13:(0.19,0.62,"left")}[x]
    dx,dy,ha=place
    ax.annotate(lab,(x,y),xytext=(x*dx,y*dy),ha=ha,va="center",fontsize=7.5,color=c,bbox=dict(boxstyle="round,pad=0.15",fc="white",ec="none",alpha=0.85))
# intrinsic PLANEX reference
sp=np.sqrt(2*np.pi*2.4e3*tau); so=np.sqrt(2*np.pi*2.1e3*tau)
ax.axhline(sp,color="0.35",lw=1.2,ls=":",label=f"PLANEX Intrinsic, 2.4 kHz Lorentzian, RIO Test Report ({sp:.2f} rad)")
ax.axhline(so,color="#7b3fa0",lw=1.2,ls="-.",label=f"ORION, Integrated Driver + LF Mod In, 2.1 kHz Measured ({so:.2f} rad); Build 2C")
xo=so/(2*np.pi*tau*K*1e-3)
ax.plot(xo,so,marker="D",color="#7b3fa0",ms=6,ls="none",zorder=6)
ax.annotate(f"ORION Equivalent ({xo*1e3:.0f} nA)",(xo,so),xytext=(xo*0.22,so*2.0),ha="left",va="center",fontsize=7.5,color="#7b3fa0",bbox=dict(boxstyle="round,pad=0.15",fc="white",ec="none",alpha=0.85),
            arrowprops=dict(arrowstyle="-",color="#7b3fa0",lw=0.7))
ax.set_xlabel(r"Driver Current Noise $i_n$ [$\mu$A RMS]")
ax.set_ylabel(r"Beat Phase Spread Over The Delay $\sigma_\varphi$ [rad RMS]")
ax.set_title("Fringe Phase Budget Versus Driver Current Noise")
ax.set_xlim(0.01,30); ax.set_ylim(0.02,60)
sec=ax.secondary_yaxis("right",functions=(lambda p:p/(2*np.pi*tau)/1e3, lambda f:f*1e3*2*np.pi*tau))
sec.set_ylabel(r"Laser Frequency Noise $\delta\nu_{\rm rms}$ [kHz]")
ax.grid(True,which="both",alpha=0.25,lw=0.5)
ax.legend(loc="lower right",frameon=True,fontsize=7)
fig.tight_layout(); fig.savefig("Build2_Noise_Budget.pdf"); fig.savefig("Build2_Noise_Budget.png",dpi=170)
print("ok")
