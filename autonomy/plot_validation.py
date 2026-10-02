"""Plot recorded localization evidence; this never feeds the running robot."""
import argparse
import json
import os
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path,nargs='?',default=Path(os.environ.get('FLEETSCOPE_RUNS', Path(__file__).resolve().parents[1]/'runs')).expanduser()/'autonomy-latest')
    args=parser.parse_args();run=args.run.resolve()
    rows=[json.loads(line) for line in (run/'pose-errors.jsonl').read_text().splitlines()]
    # Exclude later idle time and follow-up experiments from completed-route metrics.
    for filename,key in [('inspection-validation.json','full_inspection'),('autonomy-demo-report.json','all_checkpoints_reached')]:
        if (run/filename).exists():
            report=json.loads((run/filename).read_text());end=report.get('checks',{}).get(key,{}).get('details',{}).get('stamp')
            if end:
                rows=[row for row in rows if row['stamp']<=end];break
    if not rows:raise SystemExit('No time-matched pose samples recorded')
    truth=np.array([r['truth'] for r in rows]);estimate=np.array([r['estimate'] for r in rows])
    time=np.array([r['stamp'] for r in rows]);error=np.array([r['position_error_m'] for r in rows])
    prior_path=run/'config/survey-prior.npz'
    prior=np.load(prior_path if prior_path.exists() else Path(__file__).parent/'generated/survey-prior.npz')
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig=plt.figure(figsize=(12,7),facecolor='#f7f8fa');grid=fig.add_gridspec(2,2,width_ratios=[1.15,1],height_ratios=[1,1],wspace=.27,hspace=.36)
    ax=fig.add_subplot(grid[:,0]);ax.set_facecolor('#f7f8fa')
    ax.imshow(prior['data'],origin='lower',extent=(-100,100,-95,95),cmap='Greys',vmin=0,vmax=160,alpha=.5)
    ax.plot(truth[:,0],truth[:,1],color='#138b8b',lw=2.5,label='Simulator truth (evaluation only)')
    ax.plot(estimate[:,0],estimate[:,1],color='#df7925',lw=1.1,ls='--',label='Received AMCL estimate')
    goals=[('Warehouse',30,-61),('Collection shelter',-18,3),('Depot',-12,-33)]
    for name,x,y in goals:
        ax.scatter(x,y,s=45,facecolor='white',edgecolor='#253248',zorder=5)
        ax.annotate(name,(x,y),xytext=(6,7),textcoords='offset points',fontsize=9,color='#253248')
    ax.set(xlim=(-50,58),ylim=(-78,20),xlabel='Estate X (m)',ylabel='Estate Y (m)',title='Physical route and received position')
    ax.set_aspect('equal');ax.legend(loc='lower left',fontsize=8,framealpha=.95)
    ax2=fig.add_subplot(grid[0,1]);ax2.plot(time,error,color='#df7925',lw=1.)
    rms=float(np.sqrt(np.mean(error**2)));ax2.axhline(rms,color='#138b8b',ls='--',label=f'RMS: {rms:.3f} m')
    ax2.set(xlabel='Simulation time (s)',ylabel='Position error (m)',title='Timestamp-matched localization error',ylim=(0,max(.5,float(error.max())*1.15)))
    ax2.legend(fontsize=9);ax2.grid(alpha=.18)
    ax3=fig.add_subplot(grid[1,1]);ax3.axis('off')
    text=[f'{len(rows):,} matched observations',f'{np.linalg.norm(np.diff(truth[:,:2],axis=0),axis=1).sum():.1f} m of recorded travel',
          f'{rms:.3f} m RMS position error',f'{error.max():.3f} m maximum position error','',
          'Navigation: surveyed-map AMCL + Nav2','Telemetry: durable outbox, replay and acknowledgments',
          'Independent SLAM map retained as a separate layer','',run.name,'Simulation evidence; contact/noise models are illustrative.']
    ax3.text(0,1,'\n'.join(text),ha='left',va='top',linespacing=1.7,color='#253248',fontsize=10)
    fig.suptitle('FleetScope | Autonomous plantation inspection',fontsize=17,fontweight='bold',x=.08,ha='left',color='#253248')
    fig.subplots_adjust(top=.9,bottom=.1,left=.07,right=.97)
    output=run/'navigation-validation.png';fig.savefig(output,dpi=180,facecolor=fig.get_facecolor());plt.close(fig)
    print(output)

if __name__=='__main__':main()
