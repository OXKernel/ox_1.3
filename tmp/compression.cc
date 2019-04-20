// Optimized somewhat, to fully speed up,
// use a Trie as described in Sedgewick Algorithms version 4 book.
// LZW implementation, ready for use within a kernel.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
 
#include <string>
#include <map>
#include <iostream>
 
using namespace std;
 
class LZW {
    private:
    static int R;  // number of input chars
    static int L;  // number of codewords = 2^W
    static int W;  // codeword width
    static int B;  // which bit we are in
 
    public:
 
    LZW() { }
 
    static string readString(FILE *in)
    {
        string out;
        char ch = 0;
        fseek(in, 0, SEEK_END);
        size_t len = ftell(in);
        fseek(in, 0, SEEK_SET);
        fprintf(stderr,"readString:: reading in [%d] bytes\n",len);
        char *buf = new char[len];
        fread(buf,1,len,in);
        out.assign(buf,len);
        delete[] buf;
#if 0
        while(len--) {
            ch = fgetc(in);
            out += ch;
        }
#endif
        fprintf(stderr,"readString:: done reading in file\n");
        return out;
    }
 
    static void writeBit(FILE *out, bool bit)
    {
        static char ch = 0;
        static int   i = 0;
 
        if(bit) {
            //fprintf(stderr,"B [%d] bit [%d]\n",B,bit);
            ch |= (1 << (i % 8));
        }
        ++i;
        if((i  % 8) == 0) {
            //fprintf(stderr,"ch [%d]\n",ch);
            fwrite(&ch,1,sizeof(ch),out);
            ch = 0;
        }
    }
 
    static unsigned rotr(unsigned x, char r)
    {
        asm("rorl %1,%0" : "+r" (x) : "c" (r));
        return x;
    }
 
    static unsigned rotl(unsigned x, char r)
    {
        asm("roll %1,%0" : "+r" (x) : "c" (r));
        return x;
    }
 
 
    static void write(FILE *out, int x, int r)
    {
        if(r == 32) {
            fwrite(&x,1,sizeof(int),out);
            return;
        }
 
        if(r < 1 || r > 32) throw "Illegal value";
        if(x < 0 || x >= (1 << r)) throw "Illegal value";
 
        //fprintf(stderr,"x %d r %d\n",x,r);
        //B = 0;
        for(int i = 0; i < r; ++i) {
            //bool bit = ((rotl(x,(r - i - 1))) & 1) == 1;
            bool bit = (x & (1<<i)); // TODO: Isolate the bits from 0-11 from the code x.
            //fprintf(stderr,"bit [%d]",bit);
            writeBit(out,bit);
        }
    }
 
    static string longestPrefixOf(const map<string,int> &st, string input, string &prefix)
    {
        size_t length = 0;
        string match  = "";
        size_t pos1 = 0, pos2 = 0, count = 0, k = 0;
        bool first_match = true;
 
        for(map<string,int>::const_reverse_iterator i = st.rbegin();
                i != st.rend(); ++i) {
            //size_t pos = (*i).first.find(input);
            pos1 = 0, pos2 = 0, count = 0, k = 0;
            first_match = true;
            // given a, ab, abc, and matching string abcd, find abc
            // as this is the longestPrefixOf.
            for(size_t j = 0; j < (*i).first.size(); ++j,++k) {
                if((*i).first[j] == input[k]) {
                    if(first_match) {
                        pos1 = j;
                        pos2 = k;
                        first_match = false;
                        if(pos1 != 0 || pos2 != 0) {
                            // Skip this string.
                            goto CONTINUE;
                        }
                    }
                    ++count;
                }
            }
#if 0
            pos1 = input.find((*i).first);
            if(pos1 == 0) {
                count = (*i).first.size();
                string tmp = input.substr(0,count);
                if(tmp == (*i).first) {
                    pos2 = 0;
                }
            }
#endif
            if(pos1 == 0 && pos2 == 0 && count == (*i).first.size() && length < count) {
                length = count;
                match = (*i).first;
                if(count == (*i).first.size() || count == input.size()) {
                    break;
                }
            }
CONTINUE:
            ;
        }
#if 0
        fprintf(stderr,"match data: {");
        fwrite(match.data(),1,match.size(),stderr);
        fprintf(stderr,"}\n input data: {");
        fwrite(input.data(),1,input.size(),stderr);
        fprintf(stderr,"}\n");
#endif
        //fprintf(stderr,"match size [%d]\n",match.size());
        if(match == "") { fprintf(stderr,"error matching longestPrefixOf: \n"); exit(1); }
        prefix  = input.substr(0,length);
        return match; // Match should be same as prefix now.
    }
 
    static void compress(FILE *in)
    {
        string input = readString(in);
        map<string,int> st;
        string prefix = "";
#if 0
        char ch[2]={0};
        for (int i = 0; i < R; i++) {
            ch[0] = (char)i;
            st[ch]=i;
        }
#endif
        for(int i = 0; i < R; i++) {
            st[string("") + (char)i]=i;
        }
        int code = R+1;  // R is codeword for EOF
        bool doWrite = false;
 
        // output size of actual file.
        size_t out_size = input.size();
        fprintf(stderr,"out_size = %d\n",out_size);
        fwrite(&out_size,1,sizeof(out_size),stdout);
 
        while (input.size() > 0) {
            //fprintf(stderr,"size [%d]\n",input.size());
            //fprintf(stderr,"line %d\n",__LINE__);
            string s = longestPrefixOf(st,input,prefix);  // Find max prefix match s.
            if(st.find(s) == st.end()) {
                fprintf(stderr,"error symbol not found\n");
                exit(1);
            } else {
                write(stdout,st[s],W);
            }
            //fprintf(stderr,"line %d\n",__LINE__);
            int t = s.size();
            //fprintf(stderr,"line %d\n",__LINE__);
            // t [0] code [4096] input.size [3934] L [4096]
            //fprintf(stderr,"RGD: t [%d] code [%d] input.size [%d] L [%d]\n",t,code,input.size(),L);
            if (t < input.size() && code < L) {   // Add s to symbol table.
                st[input.substr(0, t + 1)] = code++; // Must look ahead.
            }
            //fprintf(stderr,"line %d\n",__LINE__);
#if 0
            string tmp = "";
            for(int i = t; i < input.size(); ++i) {
                tmp += input[i];
            }
            input = tmp;
#endif
            if(t <= input.size()) {
 
                input = input.substr(t,input.size()); // Scan past s in input.
            } else {
                input = "";
            }
            //fprintf(stderr,"line %d\n",__LINE__);
        }
        //fprintf(stderr,"line %d\n",__LINE__);
        write(stdout,R,W);
        //fprintf(stderr,"line %d\n",__LINE__);
        fflush(stdout);
        //fprintf(stderr,"line %d\n",__LINE__);
    }
 
    static bool readBoolean(FILE *in)
    {
        static char ch = 0;
        static int   i = 0;
 
#if 0
        if((B % 8) == 0) {
            ch = fgetc(in);
        }
        //bool bit = ((ch >> (B % 8)) & 1);
        bool bit = (ch & (1 << (i++ % 8)));
        //fprintf(stderr,"B [%d] (B % 8) [%d]\n",B,B % 8);
        ++B;
#endif
        if((i % 8) == 0) {
            ch = fgetc(in);
        }
        bool bit = (ch & (1 << (i++ % 8)));
        return bit;
    }
 
    static int readInt(FILE *in, int bits)
    {
        if(bits < 1 || bits > 32) throw "Illegal value";
        int x = 0;
        for(int i = 0; i < bits; ++i) {
            bool bit = readBoolean(in);
            if (bit) x |= (1 << i);
        }
        fprintf(stderr,"codeword [%d]\n",x);
        return x;
    }
 
    static void expand(FILE *in)
    {
        string *st = new string[L];
        int i; // next available codeword value
 
        // initialize symbol table with all 1-character strings
        for (i = 0; i < R; i++)
            st[i] = string("") + (char) i;
        st[i++] = ""; // (unused) lookahead for EOF
 
        size_t in_size = 0;
        fread(&in_size,1,sizeof(in_size),in);
        B = in_size;
        fprintf(stderr,"in_size [%d]\n",in_size);
 
        int codeword = readInt(in,W);
        if (codeword == R) return;           // expanded message is empty string
        string val = st[codeword];
 
        while (true) {
            B -= fwrite(val.data(),1,val.size(), stdout); // Write and decrement out stream byte count.
            if(B <= 0) break;
            codeword = readInt(in,W);
            //if(codeword == (L-1)) break; // TODO: This is due to a bug, not sure if encoding is bad or decoding is bad.
            if (codeword == R) break;
            string s = st[codeword];
            if (i == codeword) s = val + val[0];   // special case hack
            if (i < L) st[i++] = val + s[0];
            val = s;
        }
        fflush(stdout);
    }
 
    static bool creadBoolean(char **inp)
    {
        static char ch = 0;
        static int   i = 0;
        char *in = (*inp);
        if((i % 8) == 0) {
            ch = *in; // Get a byte.
            *in++;
            (*inp) = in;
        }
        bool bit = (ch & (1 << (i++ % 8)));
        return bit;
    }
 
    static int creadInt(char **in, int bits)
    {
        if(bits < 1 || bits > 32) throw "Illegal value"; // TODO: remove throw.
        int x = 0;
        for(int i = 0; i < bits; ++i) {
            bool bit = creadBoolean(in);
            if (bit) x |= (1 << i);
        }
        fprintf(stderr,"codeword [%d]\n",x);
        return x;
    }
 
    static size_t cwrite(char *val, size_t size, char *out)
    {
        size_t i = 0;
        for(i = 0; i < size; ++i) {
            *out++ = val[i]; // Stream out.
        }
        return size;
    }
 
    static size_t myexpand(char *in, char **outp)
    {
        static char st[4096][256]; // Compiler can't allocate this much on the stack for some reason. See how much this really is.
        static int sizes[4096];
 
        int i, j, k, idx, ptr; // next available codeword value
 
        fprintf(stderr,"hello world\n");
        // initalize symbol tables with null.
        for(i = 0; i < L; i++) {
            for(j = 0; j < 256; j++) {
                st[i][j] = 0;
            }
        }
 
        for(i = 0; i < L; i++) {
            sizes[i] = 0;
        }
 
        // initialize symbol table with all 1-character strings
        for (i = 0; i < R; i++) {
            st[i][0] = (char) i;
            sizes[i] = 1;
        }
        st[i][0] = '\0'; // (unused) lookahead for EOF
        sizes[i++]=1;
 
        size_t in_size = 0;
        in_size = *(size_t *)in;
        in += sizeof(size_t);
        B = in_size;
        fprintf(stderr,"in_size [%d]\n",in_size);
 
        size_t rtvl = in_size;
        (*outp) = (char *)malloc(rtvl);
        char *out = (*outp);
 
        int codeword = creadInt(&in,W);
        if (codeword == R) return 0;           // expanded message is empty string
        char *val = st[codeword];
        idx = codeword;
        size_t bytes = 0;
 
        while (true) {
            bytes = cwrite(val,sizes[idx],out); // Write and decrement out stream byte count.
            out += bytes;
            B -= bytes;
            if(B <= 0) break;
            codeword = creadInt(&in,W);
            //if(codeword == (L-1)) break; // TODO: This is due to a bug, not sure if encoding is bad or decoding is bad.
            if (codeword == R) break;
            char *s = st[codeword];
            if (i == codeword) {
                // s = val + val[0];   // special case hack
                char tmp[4096]={0};
                for(j = 0; j < sizes[idx]; ++j) {
                    tmp[j] = val[j];
                }
                tmp[j] = val[0];
                for(k = 0; k < j; ++k) {
                    s[k] = tmp[k];
                }
                sizes[codeword] = k+1;
            }
            if (i < L) {
                //st[i++] = val + s[0];
                char *tmp = st[i];
                for(j = 0; j < sizes[idx]; ++j) {
                    tmp[j] = val[j];
                }
                tmp[j] = s[0];
                sizes[i++] = j+1;
            }
            val = s;
            idx = codeword;
        }
        fflush(stdout);
        return rtvl;
    }
 
    static void myexpand(FILE *in)
    {
        static char st[4096][256]; // Compiler can't allocate this much on the stack for some reason. See how much this really is.
        static int sizes[4096];
 
        int i, j, k, idx, ptr; // next available codeword value
 
        fprintf(stderr,"hello world\n");
        // initalize symbol tables with null.
        for(i = 0; i < L; i++) {
            for(j = 0; j < 256; j++) {
                st[i][j] = 0;
            }
        }
 
        for(i = 0; i < L; i++) {
            sizes[i] = 0;
        }
 
        // initialize symbol table with all 1-character strings
        for (i = 0; i < R; i++) {
            st[i][0] = (char) i;
            sizes[i] = 1;
        }
        st[i][0] = '\0'; // (unused) lookahead for EOF
        sizes[i++]=1;
 
        size_t in_size = 0;
        fread(&in_size,1,sizeof(in_size),in);
        B = in_size;
        fprintf(stderr,"in_size [%d]\n",in_size);
 
        int codeword = readInt(in,W);
        if (codeword == R) return;           // expanded message is empty string
        char *val = st[codeword];
        idx = codeword;
 
        while (true) {
            B -= fwrite(val,1,sizes[idx],stdout); // Write and decrement out stream byte count.
            if(B <= 0) break;
            codeword = readInt(in,W);
           //if(codeword == (L-1)) break; // TODO: This is due to a bug, not sure if encoding is bad or decoding is bad.
            if (codeword == R) break;
            char *s = st[codeword];
            if (i == codeword) {
                // s = val + val[0];   // special case hack
                char tmp[4096]={0};
                for(j = 0; j < sizes[idx]; ++j) {
                    tmp[j] = val[j];
                }
                tmp[j] = val[0];
                for(k = 0; k < j; ++k) {
                    s[k] = tmp[k];
                }
                sizes[codeword] = k+1;
            }
            if (i < L) {
                //st[i++] = val + s[0];
                char *tmp = st[i];
                for(j = 0; j < sizes[idx]; ++j) {
                    tmp[j] = val[j];
                }
                tmp[j] = s[0];
                sizes[i++] = j+1;
            }
            val = s;
            idx = codeword;
        }
        fflush(stdout);
    }
};
 
int LZW::R = 256;
int LZW::L = 4096;
int LZW::W = 12;
int LZW::B = 0;
 
int main(int argc, char **argv)
{
    FILE *in = NULL;
    LZW *lzw = new LZW();
    if      (argc == 3 && !strcmp(argv[1],"-")) { in = fopen(argv[2],"r"); LZW::compress(in);}
    else if (argc == 3 && !strcmp(argv[1],"+")) {
            char *inp, *outp;
            in = fopen(argv[2],"r");
            //LZW::myexpand(in);
            fseek(in,0,SEEK_END);
            size_t bytes = ftell(in);
            fseek(in,0,SEEK_SET);
            inp = (char *)malloc(bytes);
            fread(inp,1,bytes,in); // Stream in all bytes.
            size_t rtvl = LZW::myexpand(inp,&outp);
            fwrite(outp,1,rtvl,stdout); // Stream out the results.
    }
    else {
        fprintf(stderr,"Illegal command line argument\n");
        return 1;
    }
    fclose(in);
    return 0;
}
