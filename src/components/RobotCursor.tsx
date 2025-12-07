import React, { useEffect, useRef } from 'react';

export default function RobotCursor() {
	const ref = useRef<HTMLDivElement>(null);
	useEffect(() => {
		const onMove = (e: MouseEvent) => {
			if (ref.current) {
				ref.current.style.left = `${e.clientX}px`;
				ref.current.style.top = `${e.clientY}px`;
			}
		};
		window.addEventListener('mousemove', onMove);
		return () => window.removeEventListener('mousemove', onMove);
	}, []);
	return <div ref={ref} className="robot-cursor" />;
}
