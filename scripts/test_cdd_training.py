"""Check duplicate-aware splits without TensorFlow or the real dataset."""
import json
from pathlib import Path
import tempfile
import unittest
from PIL import Image
from train_cdd_classifier import prepare


class SplitsTest(unittest.TestCase):
    def test_disjoint_reproducible_and_conflicting_labels_excluded(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)/'data'
            classes=[f'class{i}' for i in range(6)]
            config={'classes':classes,'dataset_folders':{name:name for name in classes}}
            for label,name in enumerate(classes):
                folder=root/name
                folder.mkdir(parents=True)
                for i in range(12):
                    Image.new('RGB',(4,4),(label*30,i*10,4)).save(folder/f'{i}.png')
            (root/'class0/duplicate.png').write_bytes((root/'class0/0.png').read_bytes())
            (root/'class1/conflict.png').write_bytes((root/'class0/1.png').read_bytes())
            first,summary=prepare(root,config,Path(temp)/'first',42)
            second,_=prepare(root,config,Path(temp)/'second',42)
            self.assertEqual(first,second)
            self.assertEqual(summary['duplicates_removed'],1)
            self.assertEqual(summary['conflicting_label_images_excluded'],2)
            hashes=[{row['sha256_rgb'] for row in first[name]} for name in ('train','validation','test')]
            for i in range(3):
                for j in range(i):self.assertFalse(hashes[i]&hashes[j])
            for rows in first.values():self.assertEqual({row['label'] for row in rows},set(range(6)))
            self.assertEqual(sum(map(len,first.values())),71)


if __name__=='__main__':unittest.main()
