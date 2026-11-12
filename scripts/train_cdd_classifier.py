"""Train/evaluate a six-class CDD EfficientNetB0 checkpoint for live cameras.

Exact decoded-RGB duplicates are removed before stratified 70/15/15 splitting.
Related scenes/reencoded near-duplicates can still cross splits; this is not
an event-disjoint benchmark. Dataset images are never modified.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import os
from pathlib import Path
import random
import time


def write_json(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')


def prepare(root,config,output,seed):
    from PIL import Image
    groups=defaultdict(list)
    invalid=[]
    for index,name in enumerate(config['classes']):
        folder=root/config['dataset_folders'][name]
        if not folder.is_dir():
            raise ValueError(f'Missing dataset class: {folder}')
        for path in sorted(folder.rglob('*')):
            if not path.is_file() or path.suffix.lower() not in ('.png','.jpg','.jpeg','.bmp','.webp'):
                continue
            try:
                with Image.open(path) as source:
                    image=source.convert('RGB')
                    digest=hashlib.sha256(str(image.size).encode()+image.tobytes()).hexdigest()
                groups[digest].append({'image':path.relative_to(root).as_posix(),'label':index,'sha256_rgb':digest})
            except Exception as exc:
                invalid.append({'image':str(path),'error':str(exc)})
        print('Indexed',name,flush=True)
    by_class=defaultdict(list)
    conflicts=[]
    duplicates=[]
    for digest,rows in sorted(groups.items()):
        if len({row['label'] for row in rows})!=1:
            conflicts.extend(rows)
            continue
        by_class[rows[0]['label']].append(rows[0])
        if len(rows)>1:
            duplicates.append({'kept':rows[0]['image'],'removed':[row['image'] for row in rows[1:]]})
    rng=random.Random(seed)
    splits={'train':[],'validation':[],'test':[]}
    for label in range(len(config['classes'])):
        rows=by_class[label]
        if len(rows)<10:
            raise ValueError(f'Insufficient unique valid images for class {label}')
        rng.shuffle(rows)
        n_test=max(1,round(len(rows)*.15))
        n_val=max(1,round(len(rows)*.15))
        splits['test'].extend(rows[:n_test])
        splits['validation'].extend(rows[n_test:n_test+n_val])
        splits['train'].extend(rows[n_test+n_val:])
    for rows in splits.values():
        rng.shuffle(rows)
    summary={'seed':seed,'dataset_root':str(root),'class_names':config['classes'],
             'split_counts':{name:dict(sorted(Counter(row['label'] for row in rows).items())) for name,rows in splits.items()},
             'unique_used':sum(map(len,splits.values())),'duplicates_removed':sum(len(g['removed']) for g in duplicates),
             'conflicting_label_images_excluded':len(conflicts),'invalid_images_excluded':len(invalid),
             'limitation':'Decoded-RGB duplicate disjoint; related events and near-duplicates are not known to be disjoint.'}
    write_json(output/'splits.json',splits)
    write_json(output/'split_summary.json',summary)
    write_json(output/'excluded_images.json',{'invalid':invalid,'label_conflicts':conflicts,'duplicates':duplicates})
    print(json.dumps(summary,indent=2),flush=True)
    return splits,summary


def main():
    workspace=Path(__file__).resolve().parents[1]
    config=json.loads((workspace/'src/disaster_mrta/config/disaster_classes.yaml').read_text())
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset-root',type=Path,default=workspace/config['dataset_path'])
    parser.add_argument('--output',type=Path,default=workspace/'models/cdd')
    parser.add_argument('--results',type=Path,default=workspace/'results/cdd/training')
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--batch-size',type=int,default=32)
    parser.add_argument('--head-epochs',type=int,default=30)
    parser.add_argument('--fine-tune-epochs',type=int,default=3)
    parser.add_argument('--prepare-only',action='store_true')
    parser.add_argument('--reuse-splits',action='store_true',help='Reuse the saved, unchanged dataset split inventory')
    args=parser.parse_args()
    if args.batch_size<1 or args.head_epochs<1 or args.fine_tune_epochs<0:
        parser.error('Invalid batch size or epoch count')
    os.environ.setdefault('KERAS_HOME',str(workspace/'results/cdd/keras_cache'))
    os.environ.setdefault('TF_CPP_MIN_LOG_LEVEL','1')
    started=time.monotonic()
    root=args.dataset_root.resolve()
    if args.reuse_splits:
        splits=json.loads((args.results/'splits.json').read_text())
        split_summary=json.loads((args.results/'split_summary.json').read_text())
        if split_summary['dataset_root']!=str(root) or split_summary['seed']!=args.seed or split_summary['class_names']!=config['classes']:
            raise ValueError('Saved split configuration differs; prepare splits again')
    else:
        splits,split_summary=prepare(root,config,args.results,args.seed)
    if args.prepare_only:return
    import numpy as np
    import tensorflow as tf
    from sklearn.metrics import classification_report,confusion_matrix,accuracy_score,balanced_accuracy_score,f1_score
    tf.keras.utils.set_random_seed(args.seed)
    tf.config.threading.set_intra_op_parallelism_threads(8)
    tf.config.threading.set_inter_op_parallelism_threads(4)
    devices=tf.config.list_physical_devices('GPU')
    for device in devices:
        tf.config.experimental.set_memory_growth(device,True)
    print('TensorFlow',tf.__version__,'GPUs',devices,flush=True)
    args.output.mkdir(parents=True,exist_ok=True)
    labels={name:np.array([row['label'] for row in rows],dtype=np.int32) for name,rows in splits.items()}
    def images(name,shuffle=False):
        rows=splits[name]
        ds=tf.data.Dataset.from_tensor_slices(([str(root/row['image']) for row in rows],labels[name]))
        if shuffle:ds=ds.shuffle(len(rows),seed=args.seed,reshuffle_each_iteration=True)
        def load(path,label):
            image=tf.io.decode_image(tf.io.read_file(path),channels=3,expand_animations=False)
            image.set_shape((None,None,3))
            return tf.image.resize(tf.cast(image,tf.float32),(224,224)),label
        return ds.map(load,num_parallel_calls=8,deterministic=True).batch(args.batch_size).prefetch(2)
    datasets={name:images(name) for name in splits}
    base=tf.keras.applications.EfficientNetB0(include_top=False,weights='imagenet',input_shape=(224,224,3),pooling='avg')
    base.trainable=False
    features={}
    for name,ds in datasets.items():
        print('Extracting frozen features:',name,flush=True)
        features[name]=base.predict(ds.map(lambda image,label:image),verbose=2)
    counts=np.bincount(labels['train'],minlength=6)
    weights={i:float(len(labels['train'])/(6*count)) for i,count in enumerate(counts)}
    head=tf.keras.Sequential([tf.keras.Input((1280,)),tf.keras.layers.Dropout(.25),tf.keras.layers.Dense(6,activation='softmax')],name='cdd_head')
    head.compile(optimizer=tf.keras.optimizers.Adam(1e-3),loss='sparse_categorical_crossentropy',metrics=['accuracy'])
    class BestMacro(tf.keras.callbacks.Callback):
        def __init__(self,x,y,path):
            super().__init__()
            self.x,self.y,self.path=x,y,path
            self.best=-1.
            self.wait=0
            self.saved=False
        def on_epoch_end(self,epoch,logs=None):
            prediction=np.argmax(self.model.predict(self.x,batch_size=args.batch_size,verbose=0),axis=1)
            score=float(f1_score(self.y,prediction,labels=list(range(6)),average='macro',zero_division=0))
            if logs is not None:logs['val_macro_f1']=score
            print('Validation macro F1:',round(score,5),flush=True)
            if score>self.best+1e-5:
                self.best=score
                self.wait=0
                self.model.save_weights(self.path)
                self.saved=True
            else:self.wait+=1
            if self.wait>=6:self.model.stop_training=True
    head_best=BestMacro(features['validation'],labels['validation'],args.output/'head_best.weights.h5')
    history=head.fit(features['train'],labels['train'],validation_data=(features['validation'],labels['validation']),
        class_weight=weights,epochs=args.head_epochs,batch_size=128,callbacks=[head_best],verbose=2)
    write_json(args.results/'head_history.json',history.history)
    head.load_weights(head_best.path)
    inputs=tf.keras.Input((224,224,3),name='rgb_0_255')
    outputs=head(base(inputs,training=False))
    model=tf.keras.Model(inputs,outputs,name='cdd_efficientnet_b0')
    checkpoint=args.output/'efficientnet_cdd.keras'
    model.save(checkpoint)
    if args.fine_tune_epochs:
        base.trainable=True
        for layer in base.layers[:-20]:layer.trainable=False
        # Freeze batch-normalization statistics on this small imbalanced dataset.
        for layer in base.layers:
            if isinstance(layer,tf.keras.layers.BatchNormalization):layer.trainable=False
        model.compile(optimizer=tf.keras.optimizers.Adam(1e-5),loss='sparse_categorical_crossentropy',metrics=['accuracy'])
        fine_best=BestMacro(datasets['validation'].map(lambda x,y:x),labels['validation'],args.output/'fine_best.weights.h5')
        fine_best.best=head_best.best
        fine_history=model.fit(images('train',shuffle=True),validation_data=datasets['validation'],class_weight=weights,
                              epochs=args.fine_tune_epochs,callbacks=[fine_best],verbose=2)
        write_json(args.results/'fine_tune_history.json',fine_history.history)
        if fine_best.saved:
            model.load_weights(fine_best.path)
            model.save(checkpoint)
    # Reload the exact deployable checkpoint; test data is first scored here.
    model=tf.keras.models.load_model(checkpoint,compile=False)
    probabilities=model.predict(datasets['test'].map(lambda x,y:x),verbose=2)
    if probabilities.shape!=(len(labels['test']),6) or not np.allclose(probabilities.sum(axis=1),1.,atol=1e-5):
        raise ValueError('Export does not produce six normalized probabilities')
    predicted=probabilities.argmax(axis=1)
    report={'accuracy':float(accuracy_score(labels['test'],predicted)),
        'balanced_accuracy':float(balanced_accuracy_score(labels['test'],predicted)),
        'macro_f1':float(f1_score(labels['test'],predicted,average='macro')),
        'classification_report':classification_report(labels['test'],predicted,labels=list(range(6)),target_names=config['classes'],output_dict=True,zero_division=0),
        'confusion_matrix':confusion_matrix(labels['test'],predicted,labels=list(range(6))).tolist(),
        'test_images':len(predicted),'validation_best_macro_f1':max(head_best.best,fine_best.best if args.fine_tune_epochs else -1),
        'training_seconds':time.monotonic()-started,'tensorflow_version':tf.__version__,
        'gpu_devices':[str(d) for d in devices],'scope':'Held-out CDD image classification; not Isaac camera accuracy or victim localization.'}
    write_json(args.results/'test_metrics.json',report)
    with (args.results/'test_predictions.jsonl').open('w',encoding='utf-8') as stream:
        for row,probability,prediction in zip(splits['test'],probabilities,predicted):
            stream.write(json.dumps(dict(row,predicted_class=config['classes'][prediction],probabilities=probability.tolist()))+'\n')
    metadata={'class_names':config['classes'],'input_shape':[224,224,3],'input_range':'0_255','color_order':'RGB',
              'model_output':'probabilities','architecture':'EfficientNetB0 ImageNet transfer learning',
              'class_weights':weights,'seed':args.seed,'split_summary':split_summary,
              'checkpoint_sha256':hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
              'metrics_file':str((args.results/'test_metrics.json').resolve()),'human_damage':'Area-search clue; no localized person detection'}
    write_json(checkpoint.with_suffix('.metadata.json'),metadata)
    print('SAVED',checkpoint,flush=True)
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    main()
