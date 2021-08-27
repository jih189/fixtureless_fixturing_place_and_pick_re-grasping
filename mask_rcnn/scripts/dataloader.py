import os
import numpy as np
import torch
from PIL import Image
import matplotlib.pyplot as plt

class RegraspDataset(torch.utils.data.Dataset):
    def __init__(self, root, transforms):

        self.root = root
        self.transforms = transforms

        self.imgs = list(sorted(os.listdir(os.path.join(root, "JPEGImages"))))
        self.masks = list(sorted(os.listdir(os.path.join(root, "SegmentationClass"))))


    def __getitem__(self, idx):
        # load images and masks
        img_path = os.path.join(self.root, "JPEGImages", self.imgs[idx])
        mask_path = os.path.join(self.root, "SegmentationClass", self.masks[idx])

        img = Image.open(img_path).convert("RGB")
        # note that 0 is the backgroud here
        mask = np.load(mask_path)
        # instances are encoded as different colors
        obj_ids = np.unique(mask)
        # first id is the background, so remove it
        obj_ids = obj_ids[1:]

        # split the color-encoded mask into a set
        masks = mask == obj_ids[:, None, None]

        # get bounding box coordinates for each mask
        num_objs = len(obj_ids)
        boxes = []
        for i in range(num_objs):
            pos = np.where(masks[i])
            xmin = np.min(pos[1])
            xmax = np.max(pos[1])
            ymin = np.min(pos[0])
            ymax = np.max(pos[0])
            boxes.append([xmin, ymin, xmax, ymax])

        # convert everything into a torch.Tensor
        boxes = torch.as_tensor(boxes, dtype=torch.float32)
        
        labels = torch.as_tensor(obj_ids, dtype=torch.int64)
        # labels = torch.ones((num_objs,), dtype=torch.int64)

        masks = torch.as_tensor(masks, dtype=torch.uint8)

        image_id = torch.tensor([idx])
        area = (boxes[:,3] - boxes[:,1]) * (boxes[:,2] - boxes[:,0])
        iscrowd = torch.zeros((num_objs,),dtype=torch.int64)


        target = {}
        target["boxes"] = boxes
        target["labels"] = labels
        target["masks"] = masks
        target["image_id"] = image_id
        target["area"] = area
        target["iscrowd"] = iscrowd

        if self.transforms is not None:
            img, target = self.transforms(img, target)

        return img, target

        
        # plt.imshow(masks[1])
        # plt.show()

    def __len__(self):
        return len(self.imgs)

# def main():
#     dataset = RegraspDataset('regrasp_dataset', None)
#     dataset.__getitem__(1)


# if __name__ == "__main__":
#     main()

