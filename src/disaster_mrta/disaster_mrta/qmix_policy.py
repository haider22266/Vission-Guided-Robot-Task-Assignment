"""QMIX network definition and strict checkpoint inference. No random fallback."""
from pathlib import Path


def network_classes():
    import torch
    from torch import nn
    import torch.nn.functional as F

    class Agent(nn.Module):
        def __init__(self,obs_dim,n_agents,n_actions,hidden):
            super().__init__()
            self.fc=nn.Linear(obs_dim+n_agents+n_actions,hidden)
            self.rnn=nn.GRUCell(hidden,hidden)
            self.out=nn.Linear(hidden,n_actions)

        def forward(self,x,h):
            h=self.rnn(F.relu(self.fc(x)),h)
            return self.out(h),h

    class Mixer(nn.Module):
        def __init__(self,n_agents,state_dim,hidden):
            super().__init__()
            self.n_agents,self.hidden=n_agents,hidden
            self.w1=nn.Linear(state_dim,n_agents*hidden)
            self.b1=nn.Linear(state_dim,hidden)
            self.w2=nn.Linear(state_dim,hidden)
            self.value=nn.Sequential(nn.Linear(state_dim,hidden),nn.ReLU(),nn.Linear(hidden,1))

        def forward(self,agent_q,state):
            batch=agent_q.shape[0]
            w1=F.softplus(self.w1(state)).view(batch,self.n_agents,self.hidden)
            hidden=F.elu(torch.bmm(agent_q.view(batch,1,self.n_agents),w1)+self.b1(state).view(batch,1,self.hidden))
            return (torch.bmm(hidden,F.softplus(self.w2(state)).view(batch,self.hidden,1))
                    +self.value(state).view(batch,1,1)).view(batch,1)
    return Agent,Mixer


class QmixPolicy:
    def __init__(self,path,n_agents,max_tasks):
        from .pipeline_core import observation
        if not path or not Path(path).is_file():
            raise ValueError('QMIX requires a trained checkpoint. Set qmix_checkpoint or explicitly select allocator_backend:=baseline for integration tests.')
        import torch
        checkpoint=torch.load(path,map_location='cpu',weights_only=True)
        meta=checkpoint['metadata']
        expected={'feature_schema':'area_tasks_v1','n_agents':n_agents,'max_tasks':max_tasks,'obs_dim':5+8*max_tasks}
        for key,value in expected.items():
            if meta.get(key)!=value:
                raise ValueError(f'QMIX checkpoint {key} must equal {value!r}')
        self.torch,self.observation=torch,observation
        self.n_agents,self.max_tasks=n_agents,max_tasks
        hidden=int(meta['hidden_dim'])
        Agent,Mixer=network_classes()
        self.agent=Agent(expected['obs_dim'],n_agents,max_tasks+1,hidden)
        self.mixer=Mixer(n_agents,n_agents*expected['obs_dim'],hidden)
        self.agent.load_state_dict(checkpoint['agent_state_dict'],strict=True)
        self.mixer.load_state_dict(checkpoint['mixer_state_dict'],strict=True)
        self.agent.eval()
        self.mixer.eval()
        self.hidden=torch.zeros(n_agents,hidden)
        self.previous=torch.zeros(n_agents,max_tasks+1)
        self.previous[:,0]=1

    def actions(self,fleet):
        torch=self.torch
        obs=torch.tensor(self.observation(fleet,self.max_tasks),dtype=torch.float32)
        x=torch.cat((obs,torch.eye(self.n_agents),self.previous),dim=1)
        with torch.no_grad():
            q,self.hidden=self.agent(x,self.hidden)
            masks=fleet.mask()
            masks=[row+[False]*(self.max_tasks+1-len(row)) for row in masks]
            mask=torch.tensor(masks,dtype=torch.bool)
            q=q.masked_fill(~mask,-torch.inf)
            # Centralized greedy conflict resolution retains one task per robot.
            used=set()
            result=[]
            for i in range(self.n_agents):
                for action in used:
                    q[i,action]=-torch.inf
                action=int(q[i].argmax())
                result.append(action)
                if action:
                    used.add(action)
            self.previous=torch.nn.functional.one_hot(torch.tensor(result),self.max_tasks+1).float()
        return result
