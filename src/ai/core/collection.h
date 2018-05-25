#ifndef __CORE__COLLECTION__H__
#define __CORE__COLLECTION__H__

template <typename DATA>
std::vector<DATA> list2vector(
    const std::list<DATA> & list,
    size_t vector_size
){
    assert( vector_size >= list.size() );
    std::vector<DATA> vec(list.size());
    unsigned int i = 0;
    for( const DATA & data : list ){
        vec[i] = data;
        i++;
    }
    return vec;
}

template <typename DATA>
std::vector<DATA> list2vector(
    const std::list<DATA> & list
){
    return list2vector( list, list.size() );
}

#endif
