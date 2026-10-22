/*
 * Point a ROS string field at a C string, without copying or allocating.
 *
 * Only for messages we send: the serializer only reads the string, so it
 * may point at a constant. Every string field needs data, also an empty
 * one: the serializer copies size + 1 bytes.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef DONTFALL_M4_ROS_STR_H_
#define DONTFALL_M4_ROS_STR_H_

#include <string.h>

#include <rosidl_runtime_c/string.h>

static inline void ros_str_set(rosidl_runtime_c__String *s, const char *v)
{
	s->data = (char *)v;
	s->size = strlen(v);
	s->capacity = s->size + 1;
}

#endif /* DONTFALL_M4_ROS_STR_H_ */
