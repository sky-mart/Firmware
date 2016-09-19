set(TRAINER_SRCS trainer.cpp)

px4_add_module(
	MODULE modules__trainer
	MAIN trainer
	PRIORITY "SCHED_PRIORITY_MAX-5"
	STACK_MAIN 1200
	COMPILE_FLAGS
		-Wno-type-limits
		-O3
	SRCS
		${TRAINER_SRCS}
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :