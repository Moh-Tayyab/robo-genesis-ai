
import asyncio
from sqlalchemy.ext.asyncio import create_async_engine
from sqlalchemy import text
import os
from dotenv import load_dotenv

load_dotenv()

DATABASE_URL = os.getenv("DATABASE_URL")
if not DATABASE_URL:
    print("DATABASE_URL not set")
    exit(1)

async def check_db():
    # Manipulate URL to fix asyncpg sslmode issue
    url = DATABASE_URL
    if "?" in url:
        url = url.split("?")[0]
        print("Stripped query params from URL for asyncpg compatibility")
        
    print(f"Connecting to {url.split('@')[-1]}") # Hide credentials
    engine = create_async_engine(url)
    async with engine.connect() as conn:
        print("Connected.")
        
        # Check tables
        result = await conn.execute(text("SELECT table_name FROM information_schema.tables WHERE table_schema = 'public'"))
        tables = [row[0] for row in result]
        print("Tables:", tables)

        # Try to manually create the table since Alembic is failing
        try:
            print("Attempting to CREATE table anonymous_rate_limits manually...")
            await conn.execute(text("""
                CREATE TABLE IF NOT EXISTS anonymous_rate_limits (
                    id UUID NOT NULL,
                    ip_address VARCHAR(45) NOT NULL,
                    message_count INTEGER DEFAULT 0 NOT NULL,
                    first_message_at TIMESTAMP WITH TIME ZONE DEFAULT now() NOT NULL,
                    last_message_at TIMESTAMP WITH TIME ZONE DEFAULT now() NOT NULL,
                    PRIMARY KEY (id)
                );
            """))
            await conn.execute(text("""
                CREATE UNIQUE INDEX IF NOT EXISTS ix_anonymous_rate_limits_ip_address ON anonymous_rate_limits (ip_address);
            """))
            await conn.commit()
            print("Manual CREATE TABLE succeeded.")
        except Exception as e:
            print(f"Manual CREATE TABLE failed: {e}")


        
        # Check constraints or other potential blockers?
        
        # Try to create simple table to test permissions
        try:
            await conn.execute(text("CREATE TABLE IF NOT EXISTS test_perm (id serial primary key)"))
            print("Create table test_perm succeeded")
            await conn.execute(text("DROP TABLE test_perm"))
        except Exception as e:
            print(f"Create table failed: {e}")

        # Check alembic version
        try:
            result = await conn.execute(text("SELECT * FROM alembic_version"))
            print("Alembic version:", [row[0] for row in result])
        except Exception as e:
            print(f"Read alembic_version failed: {e}")

if __name__ == "__main__":
    asyncio.run(check_db())
