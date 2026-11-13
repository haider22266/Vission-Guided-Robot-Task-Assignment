"""Visible RGB streams and whole-image predictions; never changes camera poses."""


class CameraDashboard:
    def __init__(self,records,backend):
        import omni.ui as ui
        self.records=records
        self.backend=backend
        self.providers={}
        self.labels={}
        self.tasks={}
        self.results={}
        self.frame_times={}
        self.samples={rid:0 for rid in records}
        self.window=ui.Window('Live robot cameras and CDD predictions',width=490,height=790,visible=True)
        with self.window.frame:
            with ui.VStack(spacing=6):
                ui.Label('Five robot cameras | '+backend, height=22)
                self.status=ui.Label('Waiting for RGB frames...',height=32,word_wrap=True)
                ui.Label('Whole-image area clues; no person/object localization.',height=32,word_wrap=True)
                with ui.ScrollingFrame():
                    with ui.VStack(spacing=8):
                        for rid in records:
                            provider=ui.DynamicTextureProvider('cdd_robot_camera_'+str(rid))
                            provider.set_bytes_data([35,40,45,255]*4,[2,2])
                            self.providers[rid]=provider
                            with ui.HStack(height=158,spacing=8):
                                ui.ImageWithProvider(provider,width=140,height=140,fill_policy=ui.IwpFillPolicy.IWP_STRETCH)
                                with ui.VStack(spacing=3):
                                    ui.Label('Robot %02d'%rid,height=20)
                                    self.labels[rid]=ui.Label('Waiting for camera/model...',height=78,word_wrap=True)
                                    self.tasks[rid]=ui.Label('',height=48,word_wrap=True)
        viewport=ui.Workspace.get_window('Viewport')
        if viewport is not None:self.window.dock_in(viewport,ui.DockPosition.RIGHT,.30)

    def frame(self,rid,rgb,time):
        import numpy as np
        height,width=rgb.shape[:2]
        rgba=np.empty((height,width,4),dtype=np.uint8)
        rgba[:,:,:3]=rgb
        rgba[:,:,3]=255
        self.providers[rid].set_bytes_data(rgba.reshape(-1).tolist(),[width,height])
        self.frame_times[rid]=time
        self.samples[rid]+=1
        self.refresh(rid)

    def result(self,result):
        rid=result['robot_id']
        self.results[rid]=result
        self.refresh(rid)

    def refresh(self,rid):
        result=self.results.get(rid)
        if result is None:
            prediction='Waiting for model result...'
        else:
            prediction='%s %.1f%% | result @ %.1fs\nRisk %.2f | search clue %.2f | priority %.2f'%(
                result['predicted_class'],result['confidence']*100,result['simulation_time'],
                result['hazard_risk'],result['rescue_signal'],.5+.5*max(result['hazard_risk'],result['rescue_signal']))
            if result['source']=='mock':prediction='MOCK TEST: '+prediction
        self.labels[rid].text='Camera @ %.1fs\n%s'%(self.frame_times.get(rid,0.),prediction)
        record=self.records[rid]
        self.tasks[rid].text=record['task']+'\n'+record['state']

    def update_status(self,busy):
        self.status.text='RGB streams: %d/%d | classifier %s'%(sum(count>0 for count in self.samples.values()),len(self.samples),
            'processing latest submitted batch' if busy else 'ready')
        for rid in self.records:self.refresh(rid)

    def summary(self):
        return {'enabled':True,'window_visible':self.window.visible,'displayed_frame_samples':self.samples,
                'note':'Live frame time and last prediction time are shown separately.'}

    def close(self):
        self.window.destroy()
        self.providers.clear()
