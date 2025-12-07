import React from 'react';
import { motion } from 'framer-motion';
import styles from './RoboticsCard.module.css';

interface Props {
	title: string;
	description: string;
	href: string;
}
export default function RoboticsCard({ title, description, href }: Props) {
	return (
		<motion.a
			href={href}
			className={`card ${styles.card} robotics-card`}
			whileTap={{ scale: 0.98 }}
		>
			<div className="card__body">
				<h4>{title}</h4>
				<p>{description}</p>
			</div>
		</motion.a>
	);
}
