"""Live status of symbolic tasks, displayed beside the NuRec viewport."""


class TaskPanel:
    def __init__(self,response):
        import omni.ui as ui
        self.response=response
        self.window=ui.Window('Disaster response tasks',width=420,height=270)
        with self.window.frame:
            with ui.VStack(spacing=8):
                self.summary=ui.Label('Starting tasks...',height=28)
                self.labels=[ui.Label('',height=48,word_wrap=True) for _ in response.tasks]
                ui.Label('Simulation: navigation + timed task execution.\nNo physical pickup or real inspection inference.',height=40,word_wrap=True)
        self.update()

    def update(self):
        tasks=self.response.tasks
        count=sum(t.status=='completed' for t in tasks)
        self.summary.text=f'Completed {count} / {len(tasks)} tasks'
        for label,task in zip(self.labels,tasks):
            text=f'{task.task_id}: {task.task_type} | {task.status}\nRobot: {task.assigned_robot or "unassigned"}'
            if task.status=='in_progress':
                text+=f' | {max(0,self.response.remaining.get(task.task_id,0)):.1f}s remaining'
            if task.blocked_reason:
                text+='\n'+task.blocked_reason
            label.text=text
