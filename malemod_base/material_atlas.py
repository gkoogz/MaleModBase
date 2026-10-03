"""Engine-independent atlas layout preserving native and anatomy UV islands."""
import numpy as np


def atlas_uv(uv, tile, tiles=2):
    uv=np.asarray(uv,dtype=float)
    if uv.ndim!=2 or uv.shape[1]!=2 or not np.isfinite(uv).all() or not isinstance(tiles,int) or tiles<1 or not isinstance(tile,int) or not 0<=tile<tiles:
        raise ValueError('Invalid atlas UV/tile layout')
    if np.any(uv < 0) or np.any(uv > 1):raise ValueError('Atlas requires UVs inside the original unit island')
    result=uv.copy();result[:,0]=(result[:,0]+tile)/tiles;return result


def compose_tiles(images):
    """Paste complete lossless RGBA tiles, with no source artwork resampling."""
    from PIL import Image
    if not images or any(i.size!=images[0].size for i in images):raise ValueError('Atlas tiles must have identical measured resolution')
    w,h=images[0].size
    result=Image.new('RGBA',(w*len(images),h))
    for tile,i in enumerate(images):result.paste(i.convert('RGBA'),(tile*w,0))
    return result


def bake_repeat(image, repeat):
    """Bake an explicit observed repeated sampler into a unit UV tile.

    Pixel centers use wrapped bilinear sampling. This is a material layout
    operation, not an artistic edit or an inference of a shader's settings.
    """
    from PIL import Image
    if not isinstance(repeat,int) or repeat<1:raise ValueError('Expected explicit integer repeat')
    p=np.asarray(image.convert('RGBA'));h,w=p.shape[:2]
    x=(np.arange(w)+.5)*repeat-.5;y=(np.arange(h)+.5)*repeat-.5
    xi=np.floor(x).astype(int);yi=np.floor(y).astype(int)
    result=np.empty_like(p)
    # Row blocks bound temporary storage for full-resolution source textures.
    for start in range(0,h,32):
        ids=yi[start:start+32];fy=(y[start:start+32]-ids)[:,None,None];fx=(x-xi)[None,:,None]
        a=p[ids[:,None]%h,xi[None,:]%w].astype(float)*(1-fx)+p[ids[:,None]%h,(xi[None,:]+1)%w]*fx
        b=p[(ids[:,None]+1)%h,xi[None,:]%w].astype(float)*(1-fx)+p[(ids[:,None]+1)%h,(xi[None,:]+1)%w]*fx
        result[start:start+32]=np.floor(a*(1-fy)+b*fy+.5).astype(np.uint8)
    return Image.fromarray(result,'RGBA')
