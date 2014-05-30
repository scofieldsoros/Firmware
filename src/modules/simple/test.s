	.cpu arm7tdmi
	.fpu softvfp
	.eabi_attribute 20, 1
	.eabi_attribute 21, 1
	.eabi_attribute 23, 3
	.eabi_attribute 24, 1
	.eabi_attribute 25, 1
	.eabi_attribute 26, 1
	.eabi_attribute 30, 6
	.eabi_attribute 34, 0
	.eabi_attribute 18, 4
	.file	"test.c"
	.section	.rodata
	.align	2
.LC0:
	.ascii	"b in llu= %llu\012\000"
	.text
	.align	2
	.global	main
	.type	main, %function
main:
	@ Function supports interworking.
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 1, uses_anonymous_args = 0
	stmfd	sp!, {fp, lr}
	add	fp, sp, #4
	sub	sp, sp, #8
	ldr	r3, .L2
	str	r3, [fp, #-8]
	ldr	r0, .L2+4
	ldr	r1, [fp, #-8]
	bl	printf
	mov	r0, r3
	sub	sp, fp, #4
	ldmfd	sp!, {fp, lr}
	bx	lr
.L3:
	.align	2
.L2:
	.word	123456789
	.word	.LC0
	.size	main, .-main
	.ident	"GCC: (GNU Tools for ARM Embedded Processors) 4.7.4 20130913 (release) [ARM/embedded-4_7-branch revision 202601]"
