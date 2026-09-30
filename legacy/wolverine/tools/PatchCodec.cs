using System;
using System.IO;
using System.IO.Compression;

// WBX1: magic, source length (Int64), target length (Int64), GZip XOR bytes.
// Requires the exact source file; SHA-256 checks are enforced by Install.ps1.
public static class WolverinePatchCodec {
    public static void Build(string original, string modified, string patch) {
        byte[] a = File.ReadAllBytes(original), b = File.ReadAllBytes(modified);
        byte[] delta = new byte[b.Length];
        for (int i=0; i<b.Length; i++) delta[i]=(byte)(b[i] ^ (i<a.Length ? a[i] : 0));
        using (var file = File.Create(patch)) {
            var writer = new BinaryWriter(file);
            writer.Write(new byte[]{87,66,88,49}); writer.Write((long)a.Length); writer.Write((long)b.Length); writer.Flush();
            using(var zip = new GZipStream(file, CompressionLevel.Optimal, true)) zip.Write(delta,0,delta.Length);
        }
    }
    public static void Apply(string original, string patch, string output) {
        byte[] a=File.ReadAllBytes(original);
        using(var file=File.OpenRead(patch)) {
            var reader=new BinaryReader(file);
            if(reader.ReadUInt32()!=0x31584257) throw new InvalidDataException("Unknown patch format.");
            long sourceLength=reader.ReadInt64(), targetLength=reader.ReadInt64();
            if(sourceLength!=a.LongLength || targetLength<1 || targetLength>67108864) throw new InvalidDataException("Invalid package lengths.");
            byte[] b=new byte[(int)targetLength];
            using(var zip=new GZipStream(file,CompressionMode.Decompress,true)) {
                int offset=0, count;
                while(offset<b.Length && (count=zip.Read(b,offset,b.Length-offset))>0) offset+=count;
                if(offset!=b.Length || zip.ReadByte()!=-1) throw new InvalidDataException("Patch length mismatch.");
            }
            for(int i=0;i<b.Length;i++) b[i]^=i<a.Length ? a[i] : (byte)0;
            File.WriteAllBytes(output,b);
        }
    }
}
