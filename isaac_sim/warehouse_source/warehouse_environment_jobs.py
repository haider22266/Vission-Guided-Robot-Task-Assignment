"""Task candidates drawn only from existing disaster props, never added stock."""


ENVIRONMENT_JOBS = (
    ('Recover rear-left carton', 'Rear_Left_Carton_', ('Carton', 'Tape')),
    ('Clear aisle rubble', 'Debris_', None),
    ('Recover screwdriver', 'Tool_Screwdriver_Handle_', ('Handle', 'Shaft')),
    ('Recover rear-right beam', 'Rear_Right_Beam_', None),
    ('Recover rear-right carton', 'Rear_Right_Carton_', ('Carton', 'Tape')),
)


def existing_prop_jobs(stage):
    """Return disjoint candidate groups with original scene paths.

    Selection and reservation happen in the controller. A screwdriver's shaft
    and a carton's tape are parts of the same job, not separate loose stock.
    """
    root = stage.GetPrimAtPath('/World/DisasterAftermath/Props')
    if not root:
        raise RuntimeError('Warehouse disaster props are missing')
    children = list(root.GetChildren())
    jobs = []
    seen = set()
    for robot_id, (name, prefix, companion) in enumerate(ENVIRONMENT_JOBS, 1):
        groups = []
        for prim in sorted(children, key=lambda p: str(p.GetPath())):
            if not prim.GetName().startswith(prefix):
                continue
            group = [prim]
            if companion:
                other = stage.GetPrimAtPath(str(prim.GetPath()).replace(*companion))
                if other:
                    group.append(other)
            paths = [str(p.GetPath()) for p in group]
            if any(path in seen for path in paths):
                raise RuntimeError('An environment prop appears in two robot jobs')
            seen.update(paths)
            groups.append(group)
        if not groups:
            raise RuntimeError('No existing objects for robot %d: %s' % (robot_id, name))
        jobs.append(dict(robot=robot_id, task=name, candidates=groups))
    return jobs
