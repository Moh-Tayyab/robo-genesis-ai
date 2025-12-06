"""Initial user schema

Revision ID: 001_initial_user_schema
Revises:
Create Date: 2025-12-06 11:45:00.000000

"""
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# revision identifiers
revision = '001_initial_user_schema'
down_revision = None
branch_labels = None
depends_on = None


def upgrade():
    # Create users table with background fields
    op.create_table('users',
        sa.Column('user_id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.Column('created_at', sa.DateTime(), nullable=False),
        sa.Column('updated_at', sa.DateTime(), nullable=False),
        sa.Column('email', sa.String(), nullable=True),
        sa.Column('programming_experience', sa.String(), nullable=True),
        sa.Column('os_preference', sa.String(), nullable=True),
        sa.Column('gpu_available', sa.Boolean(), nullable=True),
        sa.Column('preferred_language', sa.String(), nullable=True),
        sa.Column('learning_goals', sa.Text(), nullable=True),
        sa.Column('hardware_background', sa.String(), nullable=True),
        sa.Column('software_background', sa.String(), nullable=True),
        sa.Column('profile_completed', sa.Boolean(), nullable=False, server_default='false'),
        sa.Column('preferences', sa.String(), nullable=True),
        sa.PrimaryKeyConstraint('user_id')
    )

    # Create indexes for frequently queried fields
    op.create_index('ix_users_email', 'users', ['email'], unique=False)
    op.create_index('ix_users_programming_experience', 'users', ['programming_experience'], unique=False)
    op.create_index('ix_users_profile_completed', 'users', ['profile_completed'], unique=False)


def downgrade():
    op.drop_index('ix_users_profile_completed')
    op.drop_index('ix_users_programming_experience')
    op.drop_index('ix_users_email')
    op.drop_table('users')