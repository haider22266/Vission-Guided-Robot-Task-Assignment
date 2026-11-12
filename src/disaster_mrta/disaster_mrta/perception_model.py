"""Whole-image EfficientNet adapter; no person localization is produced."""
import math


CLASSES=('Damaged Infrastructure','Fire Disaster','Human Damage','Land Disaster','Non-Damage','Water Disaster')


def vision_result(probabilities,classes,area_id,robot_id,source='efficientnet'):
    values=[float(x) for x in probabilities]
    if len(values)!=len(classes) or set(classes)!=set(CLASSES):
        raise ValueError('Model output/class-order mismatch')
    if not all(math.isfinite(p) and 0<=p<=1 for p in values) or not .99<=sum(values)<=1.01:
        raise ValueError('Expected six normalized class probabilities')
    scores=dict(zip(classes,values))
    index=max(range(len(values)),key=values.__getitem__)
    return {'area_id':str(area_id),'robot_id':robot_id,'predicted_class':classes[index],
            'confidence':values[index],'class_scores':scores,
            'hazard_risk':max(scores[c] for c in CLASSES if c not in ('Human Damage','Non-Damage')),
            'rescue_signal':scores['Human Damage'],'source':source,
            'interpretation':'Whole-image area clue; no specific person detected or localized'}


class EfficientNetModel:
    def __init__(self,path,classes,input_range='0_255',output_type='probabilities'):
        from pathlib import Path
        if not path or not Path(path).is_file():
            raise ValueError('Set efficientnet_model to your trained Keras model, or explicitly choose perception_backend:=mock for a transport test.')
        metadata_path=Path(path).with_suffix('.metadata.json')
        if metadata_path.is_file():
            import json
            metadata=json.loads(metadata_path.read_text(encoding='utf-8'))
            if metadata.get('class_names')!=list(classes):
                raise ValueError('Checkpoint metadata class order differs from configured class_names')
            if metadata.get('input_range')!=input_range or metadata.get('model_output')!=output_type:
                raise ValueError('Checkpoint metadata preprocessing/output differs from adapter configuration')
        import tensorflow as tf
        self.tf=tf
        self.model=tf.keras.models.load_model(path,compile=False)
        if self.model.input_shape[1:]!=(224,224,3) or self.model.output_shape[-1]!=6:
            raise ValueError('Expected a 224x224 RGB model with six class outputs')
        self.classes=classes
        self.input_range=input_range
        self.output_type=output_type
        if input_range not in ('0_255','0_1') or output_type not in ('probabilities','logits'):
            raise ValueError('Unsupported preprocessing/output type')

    def predict(self,rgb):
        tf=self.tf
        x=tf.image.resize(tf.convert_to_tensor(rgb),(224,224))
        x=tf.cast(x,tf.float32)
        if self.input_range=='0_1':
            x=x/255.
        result=self.model(x[None],training=False)
        if self.output_type=='logits':
            result=tf.nn.softmax(result)
        return result.numpy()[0].tolist()
