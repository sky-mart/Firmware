set(STABLE_DUCKLING_SRCS stable_duckling.cpp)

px4_add_module(
	MODULE modules__stable_duckling
	MAIN stable_duckling
	PRIORITY "SCHED_PRIORITY_MAX-5"
	STACK_MAIN 1200
	COMPILE_FLAGS
		-Wno-type-limits
		-O3
	SRCS
		${STABLE_DUCKLING_SRCS}
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :