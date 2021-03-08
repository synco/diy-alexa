#ifndef _sample_buff_h_
#define _sample_buff_h_

#include <stdlib.h>
#include <math.h>
#include <map>

class Speaker;

class SampleBuff
{
private:
    float*  m_minbuff;
    float*  m_maxbuff;
    float   m_allmin = NAN;
    float   m_allmax = NAN;
    uint16_t m_length;
    uint16_t m_decimate;
    uint32_t m_count;
    bool    m_init = true;
    Speaker *m_speaker;

public:
    SampleBuff(uint16_t length, uint16_t decimation=10, uint32_t startcount=0) {
        m_count = startcount;
        m_length = length;
        m_decimate = decimation;        
        m_minbuff = (float*) malloc(m_length*sizeof(float));
        m_maxbuff = (float*) malloc(m_length*sizeof(float));
        std::fill_n(m_minbuff, m_length, NAN);
        std::fill_n(m_maxbuff, m_length, NAN);
    }

    void addSample(float value) {
        m_count += 1;
        if (m_count >= m_decimate*m_length) {
            std::fill_n(m_minbuff, m_length, NAN);
            std::fill_n(m_maxbuff, m_length, NAN);
            m_count = 0;
        }
        uint16_t bin = m_count / m_decimate;
        if (isnan(m_minbuff[bin]) || (value < m_minbuff[bin])) m_minbuff[bin] = value;
        if (isnan(m_maxbuff[bin]) || (value > m_maxbuff[bin])) m_maxbuff[bin] = value;
        if (isnan(m_allmin) || (value < m_allmin)) m_allmin = value;
        if (isnan(m_allmax) || (value > m_allmax)) m_allmax = value;
    }

    float* getMinBuff(void) {return m_minbuff;}
    float* getMaxBuff(void) {return m_maxbuff;}
    void getBuffMinMax(float &min, float &max) { 
        min = NAN;
        max = NAN;       
        for (uint16_t i=0; i<m_length; i++) {
            if (isnan(min) || (m_minbuff[i] < min)) min = m_minbuff[i];
            if (isnan(max) || (m_minbuff[i] > max)) max = m_minbuff[i];
        }
    }
    float  getAllMin(void) {return m_allmin;}
    float  getAllMax(void) {return m_allmax;}
    uint16_t    getLength(void) {return m_length;}
    void   setCount(uint32_t c) {m_count = c * m_decimate;}
};

#endif
