// CPU-only geometry test fallback when the desktop D3D device is unavailable.
// Never included in or used by the installed runtime.
class CpuVertexBuffer final:public IDirect3DVertexBuffer9{
 std::vector<unsigned char> bytes;ULONG refs=1;
public:
 explicit CpuVertexBuffer(UINT size):bytes(size){}
 HRESULT STDMETHODCALLTYPE QueryInterface(REFIID,void** out)override{*out=nullptr;return E_NOINTERFACE;}
 ULONG STDMETHODCALLTYPE AddRef()override{return ++refs;}
 ULONG STDMETHODCALLTYPE Release()override{ULONG n=--refs;if(!n)delete this;return n;}
 HRESULT STDMETHODCALLTYPE GetDevice(IDirect3DDevice9** out)override{*out=nullptr;return E_NOTIMPL;}
 HRESULT STDMETHODCALLTYPE SetPrivateData(REFGUID,const void*,DWORD,DWORD)override{return E_NOTIMPL;}
 HRESULT STDMETHODCALLTYPE GetPrivateData(REFGUID,void*,DWORD*)override{return E_NOTIMPL;}
 HRESULT STDMETHODCALLTYPE FreePrivateData(REFGUID)override{return E_NOTIMPL;}
 DWORD STDMETHODCALLTYPE SetPriority(DWORD)override{return 0;}
 DWORD STDMETHODCALLTYPE GetPriority()override{return 0;}
 void STDMETHODCALLTYPE PreLoad()override{}
 D3DRESOURCETYPE STDMETHODCALLTYPE GetType()override{return D3DRTYPE_VERTEXBUFFER;}
 HRESULT STDMETHODCALLTYPE Lock(UINT offset,UINT count,void** out,DWORD)override{
  if(offset>bytes.size()||(count&&offset+count>bytes.size()))return D3DERR_INVALIDCALL;
  *out=bytes.data()+offset;return S_OK;
 }
 HRESULT STDMETHODCALLTYPE Unlock()override{return S_OK;}
 HRESULT STDMETHODCALLTYPE GetDesc(D3DVERTEXBUFFER_DESC* out)override{
  memset(out,0,sizeof(*out));out->Type=D3DRTYPE_VERTEXBUFFER;out->Size=(UINT)bytes.size();return S_OK;
 }
};
