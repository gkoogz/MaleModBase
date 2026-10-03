"""Deterministic attachment color transfer from measured material samples.

Only diffuse RGB changes. UVs, alpha, normals and source detail remain intact.
Samples must come from the actual two sides of a character attachment.
"""
import numpy as np


def linear(rgb):
    p=np.asarray(rgb,dtype=float)/255
    return np.where(p<=.04045,p/12.92,((p+.055)/1.055)**2.4)


def encoded(rgb):
    p=np.clip(rgb,0,1)
    return np.floor(np.clip(np.where(p<=.0031308,p*12.92,1.055*p**(1/2.4)-.055),0,1)*255+.5).astype(np.uint8)


def sample(image, uv):
    p=np.asarray(image.convert('RGBA'));u=np.asarray(uv,dtype=float)
    if u.ndim!=2 or u.shape[1]!=2 or not np.isfinite(u).all() or np.any((u<0)|(u>1)):
        raise ValueError('Expected measured unit-island UV samples')
    x=np.minimum(p.shape[1]-1,(u[:,0]*p.shape[1]).astype(int))
    y=np.minimum(p.shape[0]-1,(u[:,1]*p.shape[0]).astype(int))
    return p[y,x,:3]


def match(image, source_samples, target_samples):
    from PIL import Image
    samples=[np.asarray(s,dtype=float) for s in [source_samples,target_samples]]
    if any(s.ndim!=2 or s.shape[1]!=3 or len(s)<3 or not np.isfinite(s).all() or np.any((s<0)|(s>255)) for s in samples):
        raise ValueError('Expected finite source and target RGB seam samples')
    med=[np.median(linear(s),axis=0) for s in samples]
    if np.any(med[0]<1e-5):raise ValueError('Source seam samples contain no usable diffuse color')
    gain=med[1]/med[0]
    p=np.asarray(image.convert('RGBA')).copy()
    for start in range(0,len(p),32):p[start:start+32,:,:3]=encoded(linear(p[start:start+32,:,:3])*gain)
    return Image.fromarray(p,'RGBA'),dict(linearRGBGain=gain.tolist(),sourceMedianLinear=med[0].tolist(),targetMedianLinear=med[1].tolist(),method='median seam samples, linear RGB multiplicative correction',alphaPreserved=True)
