from pydantic import BaseModel, EmailStr
from typing import Optional, List

class UserBase(BaseModel):
    email: EmailStr

class UserCreate(UserBase):
    password: str
    software_experience: str
    hardware_experience: str
    interests: Optional[List[str]] = []

class UserLogin(UserBase):
    password: str

class UserProfile(BaseModel):
    software_experience: str
    hardware_experience: str
    interests: List[str]

    class Config:
        orm_mode = True

class User(UserBase):
    id: int
    is_active: bool
    profile: Optional[UserProfile] = None

    class Config:
        orm_mode = True

class Token(BaseModel):
    access_token: str
    token_type: str

class TokenData(BaseModel):
    email: Optional[str] = None
